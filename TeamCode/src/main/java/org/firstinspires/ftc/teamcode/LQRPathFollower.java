package org.firstinspires.ftc.teamcode;
/**
 * LQR would be more useful for tuning turrets this year so restructure + recode
 * Pathing is a non issue so any curve works
 */

public class LQRPathFollower {

    // ---- Robot capability. MEASURE THESE, do not trust the defaults. ----
    // Drive full power in a straight line, read steady-state velocity off
    // odometry. Everything downstream is scaled by these numbers, so a wrong
    // value here mistunes the whole follower.
    private final double maxVelocity;        // in/s
    private final double maxAcceleration;    // in/s^2
    private final double maxAngularVelocity; // rad/s

    /**
     * Fraction of maxVelocity the PROFILE is allowed to ask for. This is not a
     * fudge factor, it is a real constraint with two causes:
     *
     * 1. The mecanum diagonal penalty. The wheel mixer computes vx + vy, so a
     *    45-degree diagonal saturates at |vx|+|vy| = 1, capping diagonal speed
     *    at 1/sqrt(2) = 70.7% of straight-line speed. A profile that commands
     *    100% on a diagonal is asking for a velocity the drivetrain cannot
     *    produce; the normalizer then silently scales the whole command down.
     *
     * 2. Feedback needs headroom. If feedforward alone saturates the output,
     *    the LQR correction and the rotation command get scaled away with it,
     *    so the controller cannot correct error precisely when error is worst.
     *
     * Profiling at 100% measured max produced ~10 inches of mean tracking lag
     * and 7 degrees of heading error in simulation. Lower this if you see the
     * robot lagging its reference; raise it if autos are too slow and tracking
     * is already tight.
     */
    private final double velocityHeadroom;

    private final double[][] K; // 3x3 LQR gain, solved once at construction

    // Integral trim for steady-state offsets (friction, battery sag). Plain LQR
    // has no integral action. Frozen while the output is saturated, because
    // accumulating then only stores a surge that fires when the robot unpins.
    private static final double INTEGRAL_GAIN = 0.4;
    private static final double INTEGRAL_CLAMP = 4.0;
    private static final double SATURATION_THRESHOLD = 0.99;

    private double integralX = 0, integralY = 0, integralTheta = 0;
    private boolean saturated = false;

    // Trajectory being followed, and where we are in it.
    private HolonomicPath path;
    private ArcLengthTable arcTable;
    private TrapezoidalProfile profile;
    private double startHeading, endHeading;
    private double elapsed = 0;

    /**
     * Seconds of reference PREVIEW, to cover actuation latency (loop period +
     * motor response). Zero disables it.
     *
     * This is deliberately TIME-based, not the distance-based look-ahead used
     * by pure pursuit. The difference matters: distance look-ahead picks the
     * nearest point on the path and aims at a point some inches beyond it,
     * which on a curve is a point the robot is NOT supposed to occupy, so the
     * robot cuts the corner by design. Pure pursuit accepts that because it has
     * no velocity reference and no state feedback to work with. This follower
     * has both, so previewing the reference trajectory forward in time gives
     * the anticipation without ever aiming off the path.
     *
     * Size it to measured latency. One or two loop periods (0.02 to 0.05 s) is
     * realistic. Large values re-create the corner cutting by a different route.
     */
    private double latencySeconds = 0.0;

    /** Optional voltage-level feedforward. Null keeps the normalized mixer. */
    private ChassisDynamics dynamics = null;

    static final double NOMINAL_BATTERY = 12.0;

    private double prevVxRobot = 0, prevVyRobot = 0, prevOmega = 0;
    private boolean hasPrevCommand = false;
    private Reference lastReference = null;

    /**
     * @param q  state cost. Higher = tighter tracking, more aggressive.
     * @param r  control cost. Higher = gentler, less authority used.
     * @param qTheta / rTheta  same, for heading (separate because radians and
     *                         inches are not comparable units).
     * @param dt nominal control loop period, used to discretize the model.
     */
    public LQRPathFollower(double maxVelocity, double maxAcceleration, double maxAngularVelocity,
                           double q, double r, double qTheta, double rTheta, double dt,
                           double velocityHeadroom) {
        if (maxVelocity <= 0 || maxAcceleration <= 0 || maxAngularVelocity <= 0) {
            throw new IllegalArgumentException("velocity and acceleration limits must be > 0");
        }
        if (velocityHeadroom <= 0 || velocityHeadroom > 1) {
            throw new IllegalArgumentException("velocityHeadroom must be in (0, 1]");
        }
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;
        this.velocityHeadroom = velocityHeadroom;

        // A = I, B = dt*I, both 3x3. Diagonal Q and R keep the axes decoupled,
        // which is honest: for a holonomic robot x, y and theta really are
        // independently actuatable.
        double[][] Q = {{q, 0, 0}, {0, q, 0}, {0, 0, qTheta}};
        double[][] R = {{r, 0, 0}, {0, r, 0}, {0, 0, rTheta}};
        this.K = solveLQR(dt, Q, R);
    }

    /** Sensible starting point. Tune q/r upward for tighter tracking. */
    public static LQRPathFollower withDefaults(double maxVel, double maxAccel, double maxOmega) {
        return new LQRPathFollower(maxVel, maxAccel, maxOmega,
                /*q*/ 10.0, /*r*/ 1.0, /*qTheta*/ 8.0, /*rTheta*/ 1.0, /*dt*/ 0.02,
                /*velocityHeadroom*/ 0.7);
    }

    /** Load a path and build its timing profile. Resets progress. */
    public void followPath(HolonomicPath path, double startHeading, double endHeading) {
        this.path = path;
        this.arcTable = new ArcLengthTable(path);
        this.profile = new TrapezoidalProfile(arcTable.totalLength(),
                maxVelocity * velocityHeadroom, maxAcceleration);
        this.startHeading = startHeading;
        this.endHeading = endHeading;
        this.elapsed = 0;
        this.integralX = this.integralY = this.integralTheta = 0;
        this.saturated = false;
        this.hasPrevCommand = false;
        this.prevVxRobot = this.prevVyRobot = this.prevOmega = 0;
    }

    /** Reference pose and velocity the robot should be at, at time t into the path. */
    public Reference referenceAt(double t) {
        double s = profile.positionAt(t);
        double speed = profile.velocityAt(t);
        double param = arcTable.tAtArcLength(s);

        Vec2 pos = path.pointAt(param);
        Vec2 tangent = path.tangentAt(param);

        // Heading rotates at a constant rate across the whole path. Simple and
        // predictable. If you want the robot to face along the path instead,
        // swap this for Math.atan2(tangent.y, tangent.x).
        double total = profile.duration();
        double frac = total < 1e-9 ? 1.0 : clamp01(t / total);
        double delta = normalizeAngle(endHeading - startHeading);
        double heading = startHeading + delta * frac;

        // Angular feedforward must go to ZERO outside the profile window.
        // Leaving it at delta/total after the path ends keeps commanding
        // rotation forever; feedback then balances it at a standing offset of
        // omega/k radians, which is a heading error that never converges.
        boolean withinProfile = (t > 0) && (t < total);
        double omega = withinProfile ? delta / total : 0.0;

        return new Reference(pos, tangent.times(speed), heading, omega,
                s, path.curvatureAt(param));
    }

    /**
     * One control step.
     *
     * @param measuredX/Y/Heading  field-frame pose from odometry
     * @param dt                   actual elapsed loop time
     * @return normalized wheel powers {FL, FR, BL, BR}
     */
    public double[] update(double measuredX, double measuredY, double measuredHeading, double dt) {
        return update(measuredX, measuredY, measuredHeading, dt, NOMINAL_BATTERY);
    }

    /** Convenience overload driven straight off a Localizer. */
    public double[] update(Localizer localizer, double dt, double batteryVoltage) {
        Localizer.Pose p = localizer.getPose();
        return update(p.x, p.y, p.heading, dt, batteryVoltage);
    }

    public double[] update(double measuredX, double measuredY, double measuredHeading,
                           double dt, double batteryVoltage) {
        if (path == null) return new double[]{0, 0, 0, 0};

        elapsed += dt;

        // Error is measured against the reference the robot should be tracking
        // RIGHT NOW. The latency preview is applied only to the feedforward
        // below, never here: comparing present position against a future
        // reference would manufacture error the controller then fights.
        Reference ref = referenceAt(elapsed);
        Reference ffRef = latencySeconds > 0 ? referenceAt(elapsed + latencySeconds) : ref;

        double errX = ref.position.x - measuredX;
        double errY = ref.position.y - measuredY;
        double errTheta = normalizeAngle(ref.heading - measuredHeading);

        // u = K * error. Sign matters: the LQR law is u = -K(x - x_ref), which
        // is +K(x_ref - x). Negating this gives positive feedback and the robot
        // accelerates away from the path.
        double fbX     = K[0][0] * errX + K[0][1] * errY + K[0][2] * errTheta;
        double fbY     = K[1][0] * errX + K[1][1] * errY + K[1][2] * errTheta;
        double fbTheta = K[2][0] * errX + K[2][1] * errY + K[2][2] * errTheta;

        integralX     = accumulate(integralX, errX, dt);
        integralY     = accumulate(integralY, errY, dt);
        integralTheta = accumulate(integralTheta, errTheta, dt);

        // Feedforward from the profile (optionally previewed), plus feedback
        // correction, plus integral trim.
        double vxField = ffRef.velocity.x + fbX + INTEGRAL_GAIN * integralX;
        double vyField = ffRef.velocity.y + fbY + INTEGRAL_GAIN * integralY;
        double omega   = ffRef.omega      + fbTheta + INTEGRAL_GAIN * integralTheta;

        lastReference = ref;

        if (dynamics != null) {
            return toWheelPowersViaDynamics(vxField, vyField, omega,
                    measuredHeading, dt, batteryVoltage);
        }
        return toWheelPowers(vxField, vyField, omega, measuredHeading);
    }

    /**
     * Voltage-level output path. Chassis acceleration is estimated by
     * differencing the commanded velocity between loops, which is what Ka
     * needs. It is differenced from the COMMAND, not from measured velocity,
     * because measured acceleration is the second derivative of a noisy
     * position signal and is far too noisy to feed a gain.
     */
    private double[] toWheelPowersViaDynamics(double vxField, double vyField, double omega,
                                              double heading, double dt, double batteryVoltage) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxRobot =  vxField * c + vyField * s;
        double vyRobot = -vxField * s + vyField * c;

        double ax = 0, ay = 0, alpha = 0;
        if (hasPrevCommand && dt > 1e-6) {
            ax    = (vxRobot - prevVxRobot) / dt;
            ay    = (vyRobot - prevVyRobot) / dt;
            alpha = (omega   - prevOmega)   / dt;
        }
        prevVxRobot = vxRobot; prevVyRobot = vyRobot; prevOmega = omega;
        hasPrevCommand = true;

        double[] powers = dynamics.toMotorPowers(vxRobot, vyRobot, omega,
                ax, ay, alpha, batteryVoltage);
        double rawMax = 0;
        for (double p : powers) rawMax = Math.max(rawMax, Math.abs(p));
        saturated = rawMax > SATURATION_THRESHOLD;
        return powers;
    }

    /**
     * Field-frame velocity to normalized wheel powers.
     * The rotation is not optional: odometry and the path live in the field
     * frame, but the wheels live in the robot frame. Skip it and a robot at 90
     * degrees will strafe when told to drive forward.
     */
    private double[] toWheelPowers(double vxField, double vyField, double omega, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxRobot =  vxField * c + vyField * s;
        double vyRobot = -vxField * s + vyField * c;

        // Physical units to normalized power. Without this, a modest 24 in/s
        // enters the wheel mixer as "power 24" and everything saturates.
        double vx = vxRobot / maxVelocity;
        double vy = vyRobot / maxVelocity;
        double w  = omega   / maxAngularVelocity;

        double[] p = new double[4];
        p[0] = vx + vy + w; // FL
        p[1] = vx - vy - w; // FR
        p[2] = vx - vy + w; // BL
        p[3] = vx + vy - w; // BR

        // Measure saturation on the RAW magnitude. The divisor below is floored
        // at 1.0 so this only ever scales down; comparing against the floored
        // value would report "saturated" on every single loop.
        double rawMax = 0;
        for (double v : p) rawMax = Math.max(rawMax, Math.abs(v));
        saturated = rawMax > SATURATION_THRESHOLD;

        double divisor = Math.max(1.0, rawMax);
        for (int i = 0; i < 4; i++) p[i] /= divisor;
        return p;
    }

    private double accumulate(double current, double error, double dt) {
        boolean unwinding = (current * error) < 0;
        if (saturated && !unwinding) return current;
        double next = current + error * dt;
        return Math.max(-INTEGRAL_CLAMP, Math.min(INTEGRAL_CLAMP, next));
    }

    /** True once the profile has run out of time. Pair with a pose tolerance check. */
    public boolean isFinished() {
        return profile != null && elapsed >= profile.duration();
    }

    public double duration()      { return profile == null ? 0 : profile.duration(); }
    public double elapsed()       { return elapsed; }
    public boolean isSaturated()  { return saturated; }
    public double pathLength()    { return arcTable == null ? 0 : arcTable.totalLength(); }
    public double[][] gains()     { return K; }

    /** Preview seconds for actuation latency. See field comment before raising it. */
    public void setLatencyCompensation(double seconds) {
        if (seconds < 0) throw new IllegalArgumentException("latency must be >= 0");
        if (seconds > 0.15) {
            throw new IllegalArgumentException(
                    "latency preview above 0.15s re-creates pure-pursuit corner cutting; "
                            + "measure your real loop-to-motion delay instead");
        }
        this.latencySeconds = seconds;
    }

    /**
     * Attach voltage-level feedforward. With dynamics set, update() returns
     * powers derived from Ks/Kv/Ka and live battery voltage instead of the
     * normalized mixer.
     */
    public void setChassisDynamics(ChassisDynamics d) { this.dynamics = d; }

    public boolean hasDynamics() { return dynamics != null; }

    /** Reference used on the last update(), for telemetry. Null before first call. */
    public Reference lastReference() { return lastReference; }

    /** Segment index when following a PathChain, else 0. For telemetry. */
    public int currentSegment() {
        if (!(path instanceof PathChain) || profile == null) return 0;
        double frac = profile.duration() < 1e-9 ? 1 : clamp01(elapsed / profile.duration());
        return ((PathChain) path).segmentAt(frac);
    }
    public double profiledMaxVelocity() { return maxVelocity * velocityHeadroom; }

    /** Re-anchor onto the path after a collision has pushed the robot off it. */
    public void resyncTo(double measuredX, double measuredY) {
        if (path == null) return;
        double s = arcTable.closestArcLength(path, new Vec2(measuredX, measuredY));
        elapsed = profile.timeAtPosition(s);
    }

    // ---------------------------------------------------------------
    // Discrete algebraic Riccati solve for A = I, B = dt*I (3x3 diagonal)
    // ---------------------------------------------------------------

    /**
     * Because A and B are scaled identities and Q,R are diagonal, the matrix
     * Riccati equation splits into three independent scalar equations. Solving
     * them individually is exact, fast, and far easier to read than a general
     * matrix solver, and it removes the possibility of a singular inverse.
     */
    private static double[][] solveLQR(double dt, double[][] Q, double[][] R) {
        double[][] K = new double[3][3];
        for (int i = 0; i < 3; i++) {
            K[i][i] = solveScalar(dt, Q[i][i], R[i][i]);
        }
        return K;
    }

    /**
     * Scalar DARE:  p = q + p - p^2 b^2 / (r + p b^2),  with b = dt.
     *
     * Multiplying out gives a plain quadratic in p:
     *     b^2 p^2 - q b^2 p - q r = 0
     * so it is solved EXACTLY rather than iterated. An earlier version used
     * fixed-point iteration with a convergence guard; that guard then fired on
     * perfectly legitimate inputs, because the iteration converges very slowly
     * as dt shrinks. Iterating a problem that has a closed-form solution is
     * strictly worse: slower, tolerance-dependent, and able to fail on valid
     * input.
     *
     * The discriminant is factored as sqrt(q^2 b^2 + 4qr) with a single b
     * pulled out, rather than sqrt(q^2 b^4 + 4 b^2 q r), to avoid forming b^4
     * which underflows against the 4qr term at small dt.
     *
     * The result is still CHECKED, but against the Riccati residual rather than
     * an iteration count, so a genuinely bad solve is still reported.
     */
    /** Public entry point so heading-only controllers reuse the same solver. */
    public static double scalarLqrGain(double dt, double q, double r) {
        return solveScalar(dt, q, r);
    }

    private static double solveScalar(double b, double q, double r) {
        if (b <= 0) throw new IllegalArgumentException("dt must be > 0");
        if (q <= 0) throw new IllegalArgumentException("Q entries must be > 0");
        if (r <= 0) throw new IllegalArgumentException("R entries must be > 0 (R must be positive definite)");

        double p = (q * b + Math.sqrt(q * q * b * b + 4 * q * r)) / (2 * b);

        if (Double.isNaN(p) || Double.isInfinite(p)) {
            throw new ArithmeticException("Riccati solve produced a non-finite solution");
        }

        // Residual of the DARE at the computed p. Should be zero to rounding.
        double residual = q - (p * p * b * b) / (r + p * b * b);
        if (Math.abs(residual) > 1e-6 * Math.max(1.0, q)) {
            throw new ArithmeticException(String.format(
                    "Riccati solve failed residual check (q=%g, r=%g, dt=%g, residual=%g)",
                    q, r, b, residual));
        }

        return (b * p) / (r + p * b * b);
    }

    static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

    /** Where the robot should be, and how fast, at one instant. */
    public static class Reference {
        public final Vec2 position;
        public final Vec2 velocity;
        public final double heading;
        public final double omega;
        public final double arcLength;
        public final double curvature;

        Reference(Vec2 position, Vec2 velocity, double heading, double omega,
                  double arcLength, double curvature) {
            this.position = position; this.velocity = velocity;
            this.heading = heading; this.omega = omega;
            this.arcLength = arcLength; this.curvature = curvature;
        }
    }
}

/**
 * Trapezoidal velocity profile over arc length: ramp up at max acceleration,
 * cruise at max velocity, ramp down.
 *
 * Degenerates correctly to a TRIANGULAR profile on short paths, where the robot
 * never reaches cruising speed. Getting this case wrong is a classic bug: the
 * naive formula produces a negative cruise distance and the robot commands
 * nonsense on any move under about a foot.
 */
class TrapezoidalProfile {
    private final double length, maxVel, maxAccel;
    private final double accelTime, cruiseTime, totalTime, peakVel;

    TrapezoidalProfile(double length, double maxVel, double maxAccel) {
        if (maxVel <= 0 || maxAccel <= 0) {
            throw new IllegalArgumentException("maxVel and maxAccel must be > 0");
        }
        this.length = Math.max(0, length);
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;

        double distToFullSpeed = (maxVel * maxVel) / (2 * maxAccel);

        if (2 * distToFullSpeed <= this.length) {
            // Trapezoid: there is room to reach cruising speed.
            this.peakVel = maxVel;
            this.accelTime = maxVel / maxAccel;
            this.cruiseTime = (this.length - 2 * distToFullSpeed) / maxVel;
        } else {
            // Triangle: path is too short, so peak speed is limited by distance.
            this.peakVel = Math.sqrt(maxAccel * this.length);
            this.accelTime = this.peakVel / maxAccel;
            this.cruiseTime = 0;
        }
        this.totalTime = 2 * accelTime + cruiseTime;
    }

    double duration() { return totalTime; }
    double peakVelocity() { return peakVel; }

    /** Distance travelled along the path at time t. */
    double positionAt(double t) {
        if (t <= 0) return 0;
        if (t >= totalTime) return length;

        if (t < accelTime) {
            return 0.5 * maxAccel * t * t;
        }
        double accelDist = 0.5 * maxAccel * accelTime * accelTime;
        if (t < accelTime + cruiseTime) {
            return accelDist + peakVel * (t - accelTime);
        }
        double tDecel = t - accelTime - cruiseTime;
        return accelDist + peakVel * cruiseTime
                + peakVel * tDecel - 0.5 * maxAccel * tDecel * tDecel;
    }

    /** Speed along the path at time t. */
    double velocityAt(double t) {
        if (t <= 0 || t >= totalTime) return 0;
        if (t < accelTime) return maxAccel * t;
        if (t < accelTime + cruiseTime) return peakVel;
        return peakVel - maxAccel * (t - accelTime - cruiseTime);
    }

    /** Inverse of positionAt, by bisection. Used to re-anchor after a push. */
    double timeAtPosition(double s) {
        if (s <= 0) return 0;
        if (s >= length) return totalTime;
        double lo = 0, hi = totalTime;
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            if (positionAt(mid) < s) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }
}