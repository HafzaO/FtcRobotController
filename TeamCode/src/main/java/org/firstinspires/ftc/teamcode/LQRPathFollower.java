package org.firstinspires.ftc.teamcode;
/**
 * LQR would be more useful for tuning turrets this year so restructure + recode
 * Pathing is a non issue so any curve works
 */

public class LQRPathFollower {
    private final double maxVelocity;        // in/s
    private final double maxAcceleration;    // in/s^2
    private final double maxAngularVelocity; // rad/s

    private final double velocityHeadroom;

    private final double[][] K; // 3x3 LQR gain, solved once at construction

    private static final double INTEGRAL_GAIN = 0.4;
    private static final double INTEGRAL_CLAMP = 4.0;
    private static final double SATURATION_THRESHOLD = 0.99;

    private double integralX = 0, integralY = 0, integralTheta = 0;
    private boolean saturated = false;

    private HolonomicPath path;
    private ArcLengthTable arcTable;
    private TrapezoidalProfile profile;
    private double startHeading, endHeading;
    private double elapsed = 0;

    /**
     @param q
     @param r
     @param qTheta
     @param dt
     yes you need these later so PARAM IT ISSS
     **/
    public LQRPathFollower(double maxVelocity, double maxAcceleration, double maxAngularVelocity,
                           double q, double r, double qTheta, double rTheta, double dt, double velocityHeadroom) {
       //holy i never relaized how may of these variables i added
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

        // A = I, B = dt*I, both 3x3
        double[][] Q = {{q, 0, 0}, {0, q, 0}, {0, 0, qTheta}};
        double[][] R = {{r, 0, 0}, {0, r, 0}, {0, 0, rTheta}};
        this.K = solveLQR(dt, Q, R);
    }

    public static LQRPathFollower withDefaults(double maxVel, double maxAccel, double maxOmega) {
        return new LQRPathFollower(maxVel, maxAccel, maxOmega,
                /*q*/ 10.0, /*r*/ 1.0,
                /*qTheta*/ 8.0,
                /*rTheta*/ 1.0,
                /*dt*/ 0.02,
                /*velocityHeadroom*/ 0.7);
    }

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
    }

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
     * @param measuredX/Y/Heading
     * @param dt
     * @return
     **/
    public double[] update(double measuredX, double measuredY, double measuredHeading, double dt) {
        if (path == null) return new double[]{0, 0, 0, 0};

        elapsed += dt;
        Reference ref = referenceAt(elapsed);

        double errX = ref.position.x - measuredX;
        double errY = ref.position.y - measuredY;
        double errTheta = normalizeAngle(ref.heading - measuredHeading);

        // u = K * error. Sign matters: the LQR law is u = -K(x - x_ref), which is +K(x_ref - x).
        double fbX     = K[0][0] * errX + K[0][1] * errY + K[0][2] * errTheta;
        double fbY     = K[1][0] * errX + K[1][1] * errY + K[1][2] * errTheta;
        double fbTheta = K[2][0] * errX + K[2][1] * errY + K[2][2] * errTheta;

        integralX     = accumulate(integralX, errX, dt);
        integralY     = accumulate(integralY, errY, dt);
        integralTheta = accumulate(integralTheta, errTheta, dt);

        // Feedforward from the profile, plus feedback correction, plus trim.
        double vxField = ref.velocity.x + fbX + INTEGRAL_GAIN * integralX;
        double vyField = ref.velocity.y + fbY + INTEGRAL_GAIN * integralY;
        double omega   = ref.omega      + fbTheta + INTEGRAL_GAIN * integralTheta;

        return toWheelPowers(vxField, vyField, omega, measuredHeading);
    }

    private double[] toWheelPowers(double vxField, double vyField, double omega, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxRobot =  vxField * c + vyField * s;
        double vyRobot = -vxField * s + vyField * c;

        double vx = vxRobot / maxVelocity;
        double vy = vyRobot / maxVelocity;
        double w  = omega   / maxAngularVelocity;

        double[] p = new double[4];
        p[0] = vx + vy + w; // FL
        p[1] = vx - vy - w; // FR
        p[2] = vx - vy + w; // BL
        p[3] = vx + vy - w; // BR

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

    public double duration(){
        return profile == null ? 0 : profile.duration();
    }
    public double elapsed(){
        return elapsed;
    }
    public boolean isSaturated(){
        return saturated;
    }
    public double pathLength(){
        return arcTable == null ? 0 : arcTable.totalLength();
    }
    public double[][] gains(){
        return K;
    }
    public double profiledMaxVelocity() { return maxVelocity * velocityHeadroom; }

    /** Re-anchor onto the path after a collision has pushed the robot off it. */
    public void resyncTo(double measuredX, double measuredY) {
        if (path == null) return;
        double s = arcTable.closestArcLength(path, new Vec2(measuredX, measuredY));
        elapsed = profile.timeAtPosition(s);
    }

    // Discrete algebraic Riccati solve for A = I, B = dt*I (3x3 diagonal) (yay math i hate it)

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
     */
    private static double solveScalar(double b, double q, double r) {
        if (b <= 0) throw new IllegalArgumentException("dt must be > 0");
        if (q <= 0) throw new IllegalArgumentException("Q entries must be > 0");
        if (r <= 0) throw new IllegalArgumentException("R entries must be > 0 (R must be positive definite)");

        double p = (q * b + Math.sqrt(q * q * b * b + 4 * q * r)) / (2 * b);

        if (Double.isNaN(p) || Double.isInfinite(p)) {
            throw new ArithmeticException("Riccati solve produced a non-finite solution");
        }

        double residual = q - (p * p * b * b) / (r + p * b * b);
        if (Math.abs(residual) > 1e-6 * Math.max(1.0, q)) {
            throw new ArithmeticException(String.format(
                    "Riccati solve failed residual check (q=%g, r=%g, dt=%g, residual=%g)",
                    q, r, b, residual));
        }
        //ps none of this is as fancy as it seems ftc lib has some of it, but portfolio needs the larp
        return (b * p) / (r + p * b * b);
    }

    static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    static double clamp01(double v) {
        return v < 0 ? 0 : (v > 1 ? 1 : v);
    }

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


class TrapezoidalProfile {
    /**
     * oh Mr B ur trapezoidal motion in clutch yet again
     * Everyone reading go up to him and thank him for trapezoidal motion larp
     * (cuz this could never top his)
     */
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
            // Trapezoidal motioning it!!
            this.peakVel = maxVel;
            this.accelTime = maxVel / maxAccel;
            this.cruiseTime = (this.length - 2 * distToFullSpeed) / maxVel;
        } else {
            this.peakVel = Math.sqrt(maxAccel * this.length);
            this.accelTime = this.peakVel / maxAccel;
            this.cruiseTime = 0;
        }
        this.totalTime = 2 * accelTime + cruiseTime;
    }

    //do the time boi
    double duration() {
        return totalTime;
    }

    double peakVelocity() {
        return peakVel;
    }

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

    double velocityAt(double t) {
        if (t <= 0 || t >= totalTime)
            return 0;
        if (t < accelTime)
            return maxAccel * t;
        if (t < accelTime + cruiseTime)
            return peakVel;
        return peakVel - maxAccel * (t - accelTime - cruiseTime);
    }

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