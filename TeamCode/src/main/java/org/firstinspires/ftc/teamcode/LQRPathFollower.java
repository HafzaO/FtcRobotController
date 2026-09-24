package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/// LQR Path Follower Engine
/// Uses advanced Linear Quadratic Regulator (LQR) math to guide the robot
/// along smooth trajectories, calculating corrections for friction and battery sag on the fly.

public class LQRPathFollower {

// Maximum limits of your physical robot chassis
    private final double maxV; // Top speed in inches per second
    private final double maxA; // Top acc in inches/sec^2
    private final double maxAngV; // Top spinning speed in radians per second

// Percentage governor (e.g., 0.70 means use 70% max speed for the path, leaving 30% for corrections)
    private final double vHead;

// 3x3 matrix table holding tuning gains, calculated automatically when constructed
    private final double[][] K;

// Constants to trim steady-state tracking offset drift (like dragging wheels or dropping voltage)
    private static final double INTEGRAL_GAIN = 0.4;
    private static final double INTEGRAL_CLAMP = 4.0;
    private static final double SATURATION_THRESHOLD = 0.99;

// Running tally storage boxes to track ongoing drift offsets over time
    private double integralX = 0, integralY = 0, integralTheta = 0;
    private boolean saturated = false;

// Core tracking shape references
    private HolonomicPath path;
    private ArcLengthTable arcTable;
    private TrapezoidalProfile profile;

// Target orientations
    private double startHeading, endHeading;
    private double elapsed = 0;
    private double latencySeconds = 0.0;
    private ChassisDynamics dynamics = null;

// Electricity handling constants
    static final double NOMINAL_BATTERY = 12.0;
    private double prevVxRobot = 0, prevVyRobot = 0, prevOmega = 0;
    private boolean hasPrevCommand = false;
    private Reference lastReference = null;

// Constructor: Configures properties and calculates the LQR gain matrix tables
    public LQRPathFollower(double maxVe, double maxAcc, double maxAngVe,
                           double q, double r, double qTheta, double rTheta, double dt, double veHead) {

        if (maxVe <= 0 || maxAcc <= 0 || maxAngVe <= 0) {
            throw new IllegalArgumentException("velocity and acceleration limits must be > 0");
        }
        if (veHead <= 0 || veHead > 1) {
            throw new IllegalArgumentException("velocityHeadroom must be in (0, 1]");
        }

        maxV = maxVe;
        maxA = maxAcc;
        maxAngV = maxAngVe;
        vHead = veHead;

// Build weight tables for position errors (Q) vs motor strain effort (R)
        double[][] Q = {
                {q, 0, 0},
                {0, q, 0},
                {0, 0, qTheta}
        };
        double[][] R = {
                {r, 0, 0},
                {0, r, 0},
                {0, 0, rTheta}
        };

// Run algebraic Riccati matrix solver equation to populate tracking weights
        K = solveLQR(dt, Q, R);
    }

// Helper: Instantiates standard default tuning boundaries for normal paths
    public static LQRPathFollower withDefaults(double maxVel, double maxAccel, double maxOmega) {
        return new LQRPathFollower(
                maxVel, maxAccel, maxOmega,
                10.0, 1.0, 8.0, 1.0, 0.02, 0.7
        );
    }

// Triggers a new path execution routine, clearing old error parameters
    public void followPath(HolonomicPath path, double sH, double eH) {
        this.path = path;
        arcTable = new ArcLengthTable(path);

// Generates a speed blueprint profile based on path length and limitations
        profile = new TrapezoidalProfile(arcTable.totalLength(), maxV * vHead, maxA);

        this.sH = sH;
        this.eH = eH;
        elapsed = 0;

// Reset old integral calculations so history from last path doesn't leak into this one
        iX = integralY = integralTheta = 0;
        saturated = false;
        hasPrevCommand = false;
        prevVxRobot = prevVyRobot = prevOmega = 0;
    }

// Calculates exactly where the robot SHOULD be at time marker 't'
    public Reference referenceAt(double t) {
        double s = profile.positionAt(t);
        double speed = profile.velocityAt(t);
        double param = arcTable.tAtArcLength(s);

        Vec2 pos = path.pointAt(param);
        Vec2 tangent = path.tangentAt(param);

        double total = profile.duration();
        double frac = total < 1e-9 ? 1.0 : clamp01(t / total);
        double delta = normalizeAngle(endHeading - startHeading);
        double heading = startHeading + delta * frac;

        boolean withinProfile = (t > 0) && (t < total);
        double omega = withinProfile ? delta / total : 0.0;

        return new Reference(pos, tangent.times(speed), heading, omega, s, path.curvatureAt(param));
    }

// Master update method: called dozens of times a second to find wheel power levels
    public double[] update(double measuredX, double measuredY, double measuredHeading, double dt) {
        return update(measuredX, measuredY, measuredHeading, dt, NOMINAL_BATTERY);
    }

    public double[] update(Localizer localizer, double dt, double batteryVoltage) {
        Localizer.Pose p = localizer.getPose();
        return update(p.x, p.y, p.heading, dt, batteryVoltage);
    }

    public double[] update(double measuredX, double measuredY, double measuredHeading, double dt, double batteryVoltage) {
        if (path == null) {
            return new double[]{0, 0, 0, 0};
        }

        elapsed += dt;
        Reference ref = referenceAt(elapsed);

// If lookahead preview latency compensation is enabled, fetch future targets for feedforward push
        Reference ffRef = latencySeconds > 0 ? referenceAt(elapsed + latencySeconds) : ref;

// Measure errors between target state vs real location from sensors
        double errX = ref.position.x - measuredX;
        double errY = ref.position.y - measuredY;
        double errTheta = normalizeAngle(ref.heading - measuredHeading);

// Multiply errors across matrix coefficients to extract correction demands
        double fbX = K[0][0] * errX + K[0][1] * errY + K[0][2] * errTheta;
        double fbY = K[1][0] * errX + K[1][1] * errY + K[1][2] * errTheta;
        double fbTheta = K[2][0] * errX + K[2][1] * errY + K[2][2] * errTheta;

// Accumulate runtime background error integration logs
        integralX = accumulate(integralX, errX, dt);
        integralY = accumulate(integralY, errY, dt);
        integralTheta = accumulate(integralTheta, errTheta, dt);

// Blend perfect curve path speed with error feedback adjustments
        double vxField = ffRef.velocity.x + fbX + INTEGRAL_GAIN * integralX;
        double vyField = ffRef.velocity.y + fbY + INTEGRAL_GAIN * integralY;
        double omega   = ffRef.omega + fbTheta + INTEGRAL_GAIN * integralTheta;

        lastReference = ref;

// Process outputs through voltage/acceleration model if dynamics file is active
        if (dynamics != null) {
            return toWheelPowersViaDynamics(vxField, vyField, omega, measuredHeading, dt, batteryVoltage);
        }
        return toWheelPowers(vxField, vyField, omega, measuredHeading);
    }

// Maps field movements through full velocity and acceleration calculations
    private double[] toWheelPowersViaDynamics(double vxField, double vyField, double omega, double heading, double dt, double batteryVoltage) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxRobot =  vxField * c + vyField * s;
        double vyRobot = -vxField * s + vyField * c;
        double ax = 0, ay = 0, alpha = 0;

// Derives historical acceleration markers by observing change over loop intervals
        if (hasPrevCommand && dt > 1e-6) {
            ax = (vxRobot - prevVxRobot) / dt;
            ay = (vyRobot - prevVyRobot) / dt;
            alpha = (omega - prevOmega) / dt;
        }

        prevVxRobot = vxRobot;
        prevVyRobot = vyRobot;
        prevOmega = omega;
        hasPrevCommand = true;

        double[] powers = dynamics.toMotorPowers(vxRobot, vyRobot, omega, ax, ay, alpha, batteryVoltage);

        double rawMax = 0;
        for (double p : powers) {
            rawMax = Math.max(rawMax, Math.abs(p));
        }

// If wheels are pinned to top threshold power limits, flag saturation to pause integration tracking
        saturated = rawMax > SATURATION_THRESHOLD;
        return powers;
    }

// Direct geometric wheel power converter (Kinematic mixer mapping)
    private double[] toWheelPowers(double vxField, double vyField, double omega, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxRobot =  vxField * c + vyField * s;
        double vyRobot = -vxField * s + vyField * c;

        double vx = vxRobot / maxV;
        double vy = vyRobot / maxV;
        double w  = omega / maxAngV;

        double[] p = new double[4];
        p[0] = vx + vy + w; // Front Left
        p[1] = vx - vy - w; // Front Right
        p[2] = vx - vy + w; // Back Left
        p[3] = vx + vy - w; // Back Right

        double rawMax = 0;
        for (double v : p) {
            rawMax = Math.max(rawMax, Math.abs(v));
        }
        saturated = rawMax > SATURATION_THRESHOLD;
// Proportional scale reducer: preserves travel direction intent if power exceeds bounds
        double divisor = Math.max(1.0, rawMax);
        for (int i = 0; i < 4; i++) {
            p[i] /= divisor;
        }
        return p;
    }

//Tally function tracking baseline offsets. Freezes tracking if system hits a saturation limit
    private double accumulate(double current, double error, double dt) {
        boolean unwinding = (current * error) < 0;
        if (saturated && !unwinding) {
            return current;
        }
        double next = current + error * dt;
        return Math.max(-INTEGRAL_CLAMP, Math.min(INTEGRAL_CLAMP, next));
    }

//Returns true when path profile runtime durations are exhausted
    public boolean isFinished() {
        return profile != null && elapsed >= profile.duration();
    }

    public double duration() {
        return profile == null ? 0 : profile.duration();
    }

    public double elapsed() {
        return elapsed;
    }

    public boolean isSaturated() {
        return saturated;
    }

    public double pathLength() {
        return arcTable == null ? 0 : arcTable.totalLength();
    }

    public double[][] gains() {
        return K;
    }

// Sets preview lookahead margins to optimize feed timing
    public void setLatencyCompensation(double seconds) {
        if (seconds < 0) {
            throw new IllegalArgumentException("latency must be >= 0");
        }
        if (seconds > 0.15) {
            throw new IllegalArgumentException("latency preview too high, measure real loop delay instead");
        }
        latencySeconds = seconds;
    }

    public void setChassisDynamics(ChassisDynamics d) {
        dynamics = d;
    }

    public boolean hasDynamics() {
        return dynamics != null;
    }

    public Reference lastReference() {
        return lastReference;
    }

    public int currentSegment() {
        if (!(path instanceof PathChain) || profile == null) {
            return 0;
        }
        double frac = profile.duration() < 1e-9 ? 1 : clamp01(elapsed / profile.duration());
        return ((PathChain) path).segmentAt(frac);
    }

    public double profiledMaxV() {
        return maxV * vHead;
    }

// Snaps tracking timers forward if path elements fall far out of alignment bounds
    public void resyncTo(double measuredX, double measuredY) {
        if (path == null) {
            return;
        }
        double s = arcTable.closestArcLength(path, new Vec2(measuredX, measuredY));
        elapsed = profile.timeAtPosition(s);
    }

// Runs algebraic Riccati matrix solver equations for 3x3 grid layouts
    private static double[][] solveLQR(double dt, double[][] Q, double[][] R) {
        double[][] K = new double[3][3];
        for (int i = 0; i < 3; i++) {
            K[i][i] = solveScalar(dt, Q[i][i], R[i][i]);
        }
        return K;
    }

    public static double scalarLqrGain(double dt, double q, double r) {
        return solveScalar(dt, q, r);
    }

    private static double solveScalar(double b, double q, double r) {
        if (b <= 0) throw new IllegalArgumentException("dt must be > 0");
        if (q <= 0) throw new IllegalArgumentException("Q must be > 0");
        if (r <= 0) throw new IllegalArgumentException("R must be > 0 (R must be +)");

        double p = (q * b + Math.sqrt(q * q * b * b + 4 * q * r)) / (2 * b);
        if (Double.isNaN(p) || Double.isInfinite(p)) {
            throw new ArithmeticException("Riccati solve produced a non-finite solution");
        }

        double residual = q - (p * p * b * b) / (r + p * b * b);
        if (Math.abs(residual) > 1e-6 * Math.max(1.0, q)) {
            throw new ArithmeticException(String.format("Riccati solve failed residual check"));
        }
        return (b * p) / (r + p * b * b);
    }

// Keeps angles wrapped clean inside standard field boundary rules (-180 to +180 deg)
    static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

// Clips metrics safely within 0.0 and 1.0 limits
    static double clamp01(double v) {
        return v < 0 ? 0 : (v > 1 ? 1 : v);
    }
///Storable reference state data box tracking target properties for one tick frame
    public static class Reference {
        public final Vec2 p;
        public final Vec2 v;
        public final double h;
        public final double omega;
        public final double arcL;
        public final double curv;

        Reference(Vec2 position, Vec2 velocity, double heading, double o, double arcLength, double curvature) {
            p = position;
            v = velocity;
            h = heading;
            omega = o;
            arcL = arcLength;
            curv = curvature;
        }
    }
}


/// Velocity motion curve generator shape profile tool.
/// Handles slow-acc-coast-decelerate calculations (Trapezoids and Triangles)
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

        // Physics formula: v^2 / 2a to map spatial layout limits
        double distToFullSpeed = (maxVel * maxVel) / (2 * maxAccel);

        if (2 * distToFullSpeed <= this.length) {
            // Trapezoid path shape configuration
            this.peakVel = maxVel;
            this.accelTime = maxVel / maxAccel;
            this.cruiseTime = (this.length - 2 * distToFullSpeed) / maxVel;
        } else {
            // Triangle path shape configuration (Path is too short to hit top cruising speeds safely)
            this.peakVel = Math.sqrt(maxAccel * this.length);
            this.accelTime = this.peakVel / maxAccel;
            this.cruiseTime = 0;
        }
        this.totalTime = 2 * accelTime + cruiseTime;
    }

    double duration() {
        return totalTime;
    }

    double peakV() {
        return peakVel;
    }

    // Evaluates target relative position along curve progress profiles
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
        return accelDist + peakVel * cruiseTime + peakVel * tDecel - 0.5 * maxAccel * tDecel * tDecel;
    }

    // Evaluates target relative velocity along curve progress profiles
    double vAt(double t) {
        if (t <= 0 || t >= totalTime) return 0;
        if (t < accelTime) return maxAccel * t;
        if (t < accelTime + cruiseTime) return peakVel;
        return peakVel - maxAccel * (t - accelTime - cruiseTime);
    }

    // Uses binary searches to isolate precise lookup timeline frames for coordinate properties
    double timeAtPosition(double s) {
        if (s <= 0) return 0;
        if (s >= length) return totalTime;
        double lo = 0, hi = totalTime;

        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            if (positionAt(mid) < s) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return 0.5 * (lo + hi);
    }
}

