package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Hardware adapters for Localizer.
 *
 * READ THIS BEFORE TRUSTING THESE CLASSES.
 * The driver APIs for Pinpoint and OTOS have changed across SDK and vendor
 * releases (Pinpoint in particular moved into the SDK as
 * com.qualcomm.hardware.gobilda.GoBildaPinpointDriver after previously being a
 * file teams pasted into TeamCode). The method names below reflect the current
 * published drivers, but VERIFY them against the version you actually have.
 *
 * The adapters are isolated in this one file on purpose: if a method name is
 * wrong you get a compile error here, in eight lines of glue, rather than a
 * silent runtime misbehaviour buried in the control loop. Everything else in
 * the stack talks only to the Localizer interface and is unaffected.
 *
 * Both adapters are commented out rather than shipped live, because this file
 * must compile for teams who own neither device. Uncomment the one you use.
 */
public final class LocalizerAdapters {

    private LocalizerAdapters() { }

    /*
     * ---------------------------------------------------------------
     * goBILDA PINPOINT
     *
     * CRITICAL UNIT NOTE: getPosition() reports MILLIMETRES. Feeding those
     * straight into this inches-based stack does not throw, it just drives the
     * robot 25.4x too far. The conversion below is the only place that matters.
     *
     * Setup order that actually works:
     *   pinpoint.setOffsets(xOffsetMm, yOffsetMm);
     *   pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
     *   pinpoint.resetPosAndIMU();   // robot MUST be stationary
     *   sleep(300);                  // wait for IMU calibration
     * Do not put the Pinpoint on I2C port 0; that port is reserved.
     * ---------------------------------------------------------------

    public static class PinpointLocalizer implements Localizer {
        private final GoBildaPinpointDriver pinpoint;
        private Pose pose = new Pose(0, 0, 0);

        public PinpointLocalizer(HardwareMap hw, String name) {
            pinpoint = hw.get(GoBildaPinpointDriver.class, name);
        }

        @Override public void update() {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();
            pose = new Pose(
                Localizer.mmToInches(p.getX(DistanceUnit.MM)),
                Localizer.mmToInches(p.getY(DistanceUnit.MM)),
                p.getHeading(AngleUnit.RADIANS));
        }

        @Override public Pose getPose() { return pose; }

        @Override public Velocity getVelocity() {
            Pose2D v = pinpoint.getVelocity();
            return new Velocity(
                Localizer.mmToInches(v.getX(DistanceUnit.MM)),
                Localizer.mmToInches(v.getY(DistanceUnit.MM)),
                v.getHeading(AngleUnit.RADIANS));
        }

        @Override public void setPose(Pose p) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.MM,
                Localizer.inchesToMm(p.x), Localizer.inchesToMm(p.y),
                AngleUnit.RADIANS, p.heading));
        }
    }
    */

    /*
     * ---------------------------------------------------------------
     * SPARKFUN OTOS
     *
     * OTOS reports in whatever unit it was last configured with, so the units
     * are SET explicitly at construction rather than assumed. Configuring for
     * inches and radians makes it match this stack's contract natively.
     *
     * calibrateImu() and resetTracking() both require the robot to be still.
     * setOffset() describes where the sensor sits relative to robot centre.
     * ---------------------------------------------------------------

    public static class OtosLocalizer implements Localizer {
        private final SparkFunOTOS otos;
        private Pose pose = new Pose(0, 0, 0);

        public OtosLocalizer(HardwareMap hw, String name) {
            otos = hw.get(SparkFunOTOS.class, name);
            otos.setLinearUnit(DistanceUnit.INCH);
            otos.setAngularUnit(AngleUnit.RADIANS);
            otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
            otos.setLinearScalar(1.0);   // tune from a measured push test
            otos.setAngularScalar(1.0);
            otos.calibrateImu();
            otos.resetTracking();
        }

        @Override public void update() {
            SparkFunOTOS.Pose2D p = otos.getPosition();
            pose = new Pose(p.x, p.y, p.h);
        }

        @Override public Pose getPose() { return pose; }

        @Override public Velocity getVelocity() {
            SparkFunOTOS.Pose2D v = otos.getVelocity();
            return new Velocity(v.x, v.y, v.h);
        }

        @Override public void setPose(Pose p) {
            otos.setPosition(new SparkFunOTOS.Pose2D(p.x, p.y, p.heading));
        }
    }
    */

    /**
     * Two dead-wheel pods plus a separate IMU for heading.
     *
     * Two pods alone CANNOT observe heading. If you are reading only two pods
     * and no IMU, theta is unobservable and no controller tuning recovers it.
     * This class therefore requires the IMU supplier and will not pretend
     * otherwise.
     *
     * The pose integration itself is left to you because it depends on your pod
     * geometry (offsets, ticks per inch, direction signs) and getting those
     * wrong produces a pose that drifts plausibly rather than failing loudly.
     */
    public static abstract class TwoPodImuLocalizer implements Localizer {
        protected double x, y, heading;

        /** Pod travel since last call, in inches: {forward, strafe}. */
        protected abstract double[] readPodDeltasInches();

        /** Absolute heading from the IMU, radians, CCW positive from +x. */
        protected abstract double readImuHeadingRadians();

        @Override public void update() {
            double newHeading = readImuHeadingRadians();
            double[] d = readPodDeltasInches();

            // Midpoint heading over the interval. Using the start or end
            // heading instead accumulates a systematic arc error on every turn,
            // which is the classic "drives straight fine, drifts after every
            // rotation" symptom.
            double mid = heading + LQRPathFollower.normalizeAngle(newHeading - heading) / 2.0;
            double c = Math.cos(mid), s = Math.sin(mid);

            x += d[0] * c - d[1] * s;
            y += d[0] * s + d[1] * c;
            heading = newHeading;
        }

        @Override public Pose getPose() { return new Pose(x, y, heading); }

        /** No hardware velocity source; follower falls back to differencing. */
        @Override public Velocity getVelocity() { return null; }

        @Override public void setPose(Pose p) {
            x = p.x; y = p.y; heading = p.heading;
        }
    }
}
