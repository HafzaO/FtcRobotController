package org.firstinspires.ftc.teamcode;

/**
 * Pose source abstraction, so the follower never knows or cares what hardware
 * is underneath it.
 *
 * CONTRACT, and every implementation must honour it exactly:
 *   x, y     INCHES, field frame, origin at field centre, +x right, +y up
 *   heading  RADIANS, CCW positive, measured from +x
 *
 * The contract is spelled out because the two most common localizers do NOT
 * natively speak it. goBILDA Pinpoint reports millimetres. SparkFun OTOS
 * reports whatever unit you last configured it with. A unit mismatch here does
 * not throw, it just drives the robot 25.4x too far, so each adapter converts
 * at its boundary and nowhere else.
 */
public interface Localizer {

    /** Read hardware once per control loop, before any getPose() call. */
    void update();

    /** Latest pose in the contract units above. */
    Pose getPose();

    /**
     * Field-frame velocity in in/s and rad/s.
     * Implementations WITHOUT a hardware velocity output should return null,
     * and the follower will fall back to filtered finite differencing. Returning
     * a fabricated zero instead would silently disable velocity feedback.
     */
    Velocity getVelocity();

    /** Force the estimator to a known pose, e.g. at auto start. */
    void setPose(Pose pose);

    final class Pose {
        public final double x, y, heading;
        public Pose(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
        @Override public String toString() {
            return String.format("(%.1f, %.1f, %.1f deg)", x, y, Math.toDegrees(heading));
        }
    }

    final class Velocity {
        public final double vx, vy, omega;
        public Velocity(double vx, double vy, double omega) {
            this.vx = vx; this.vy = vy; this.omega = omega;
        }
    }

    // ---------------------------------------------------------------
    // Unit helpers. Kept here so conversions live in one auditable place.
    // ---------------------------------------------------------------

    double MM_PER_INCH = 25.4;

    static double mmToInches(double mm) { return mm / MM_PER_INCH; }
    static double inchesToMm(double in) { return in * MM_PER_INCH; }
}

/**
 * Simulation / testing localizer. Also useful for bench-running an auto with
 * no hardware attached, which is how the follower tests run.
 */
class SimulatedLocalizer implements Localizer {
    private Pose pose;
    private Velocity velocity = new Velocity(0, 0, 0);

    SimulatedLocalizer(double x, double y, double heading) {
        pose = new Pose(x, y, heading);
    }

    @Override public void update() { }
    @Override public Pose getPose() { return pose; }
    @Override public Velocity getVelocity() { return velocity; }
    @Override public void setPose(Pose p) { pose = p; }

    void inject(double x, double y, double heading, double vx, double vy, double omega) {
        pose = new Pose(x, y, heading);
        velocity = new Velocity(vx, vy, omega);
    }
}
