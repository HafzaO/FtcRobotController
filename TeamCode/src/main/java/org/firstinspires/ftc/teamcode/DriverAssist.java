package org.firstinspires.ftc.teamcode;

/**
 * Teleop driver assistance. Deliberately NOT the path-following stack.
 *
 * Drivers need the stick to feel connected to the robot. Running joystick input
 * through a trajectory generator adds latency and takes authority away at
 * exactly the moment a driver is reacting to something the path planner cannot
 * see. So translation stays raw, and the LQR is repurposed for the one thing
 * humans are measurably bad at: holding a precise heading while translating.
 *
 * THE AUTHORITY BUDGET, which is the part that is easy to get wrong.
 * The mecanum mixer sums translation and rotation into each wheel. If the
 * driver is already at full stick, there is no headroom left, and the heading
 * correction gets scaled away by output normalization precisely when the robot
 * is moving fastest and heading error grows quickest. Snap-to-angle then feels
 * like it "stops working at speed", which reads as a bug but is arithmetic.
 *
 * This class fixes an explicit budget: heading gets a reserved slice of output
 * authority, and translation is scaled to fit in what remains. The driver loses
 * a little top speed while the assist is active. That trade is stated out loud
 * rather than hidden, because the alternative is an assist that silently fails
 * under load.
 */
public class DriverAssist {

    private final double headingGain;
    private final double maxAngularVelocity;
    private final double headingAuthority;

    private Double lockedHeading = null;

    /**
     * @param q, r            LQR cost on heading error vs control effort.
     *                        Gain works out near sqrt(q/r); start q=8, r=1.
     * @param dt              nominal loop period
     * @param headingAuthority fraction of output reserved for rotation while a
     *                        lock is active. 0.3 is a reasonable start: snappy
     *                        correction, and the driver keeps 70% of top speed.
     */
    public DriverAssist(double q, double r, double dt,
                        double maxAngularVelocity, double headingAuthority) {
        if (headingAuthority <= 0 || headingAuthority >= 1) {
            throw new IllegalArgumentException("headingAuthority must be in (0, 1)");
        }
        if (maxAngularVelocity <= 0) throw new IllegalArgumentException("maxAngularVelocity must be > 0");
        this.headingGain = LQRPathFollower.scalarLqrGain(dt, q, r);
        this.maxAngularVelocity = maxAngularVelocity;
        this.headingAuthority = headingAuthority;
    }

    public static DriverAssist withDefaults(double maxAngularVelocity) {
        return new DriverAssist(8.0, 1.0, 0.02, maxAngularVelocity, 0.30);
    }

    /** Lock to a field angle in radians. Call on a button press. */
    public void lockHeading(double radians) { lockedHeading = LQRPathFollower.normalizeAngle(radians); }

    /** Lock to the nearest of N evenly spaced field angles. */
    public void snapToNearest(double currentHeading, int divisions) {
        if (divisions < 1) throw new IllegalArgumentException("divisions must be >= 1");
        double step = 2 * Math.PI / divisions;
        lockedHeading = LQRPathFollower.normalizeAngle(Math.round(currentHeading / step) * step);
    }

    public void release() { lockedHeading = null; }
    public boolean isLocked() { return lockedHeading != null; }
    public Double lockedHeading() { return lockedHeading; }

    /**
     * @param stickX,stickY  driver translation, already in FIELD frame and
     *                       already deadbanded, each in [-1, 1]
     * @param stickTurn      manual rotation in [-1, 1], used only when unlocked
     * @param heading        measured heading, radians
     * @return {FL, FR, BL, BR} powers
     */
    public double[] update(double stickX, double stickY, double stickTurn, double heading) {
        double rotation;
        double translationScale;

        if (lockedHeading == null) {
            rotation = stickTurn;
            translationScale = 1.0;
        } else {
            double err = LQRPathFollower.normalizeAngle(lockedHeading - heading);
            double omega = headingGain * err;                   // rad/s command
            rotation = omega / maxAngularVelocity;              // normalized
            rotation = Math.max(-headingAuthority, Math.min(headingAuthority, rotation));
            // Reserve the budget whether or not the correction currently needs
            // it, so behaviour does not change the instant error appears.
            translationScale = 1.0 - headingAuthority;
        }

        // Field frame to robot frame. Same rotation as the autonomous path.
        double c = Math.cos(heading), s = Math.sin(heading);
        double vx = ( stickX * c + stickY * s) * translationScale;
        double vy = (-stickX * s + stickY * c) * translationScale;

        double[] p = new double[]{
                vx + vy + rotation,
                vx - vy - rotation,
                vx - vy + rotation,
                vx + vy - rotation
        };

        double rawMax = 0;
        for (double v : p) rawMax = Math.max(rawMax, Math.abs(v));
        double divisor = Math.max(1.0, rawMax);
        for (int i = 0; i < 4; i++) p[i] /= divisor;
        return p;
    }

    /**
     * Build a short path from the robot's current pose to a scoring target, for
     * a hold-to-score macro.
     *
     * Handle offsets are placed along each end's heading so the robot leaves and
     * arrives facing sensibly instead of crabbing sideways into the structure.
     * Handle length scales with distance, since a fixed handle produces a wild
     * curve on short moves and a nearly straight one on long moves.
     *
     * Read the caveat: this generates geometry from live odometry and drives it
     * immediately, with no field-element collision checking whatsoever. Bind it
     * to hold-to-run, never toggle, so releasing the button always returns
     * control instantly.
     */
    public static HolonomicPath scoreMacroPath(double curX, double curY, double curHeading,
                                               double targetX, double targetY, double targetHeading) {
        double dist = Math.hypot(targetX - curX, targetY - curY);
        double handle = Math.max(4.0, dist * 0.4);
        return new CubicBezierPath(
                new Vec2(curX, curY),
                new Vec2(curX + Math.cos(curHeading) * handle,
                        curY + Math.sin(curHeading) * handle),
                new Vec2(targetX - Math.cos(targetHeading) * handle,
                        targetY - Math.sin(targetHeading) * handle),
                new Vec2(targetX, targetY));
    }
}


