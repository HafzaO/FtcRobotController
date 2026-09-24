package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A collection of hardware adapters that bridge real sensors to our Localizer rules.
 */
public final class LocalizerAdapters {

    // Stops anyone from accidentally creating a blank copy of this class
    private LocalizerAdapters() { }

    /**
     * Adapter for the goBILDA Pinpoint Odometry Computer.
     * Natively handles high-speed tracking on its own onboard coprocessor chip.
     */
    public static class PinpointLocalizer implements Localizer {
        private final GoBildaPinpointDriver pinpoint;
        private Pose currentPose = new Pose(0, 0, 0);

        // Connects & sets up the hardware when auto starts
        public PinpointLocalizer(HardwareMap hw, String name) {
            pinpoint = hw.get(GoBildaPinpointDriver.class, name);

            // Physical offsets in mm from the center of the robot to the pods [X, Y]
            // TODO: Replace 15.0 & -50.0 with your team's real measured distances!
            pinpoint.setOffsets(15.0, -50.0);

            // Tells hub its using 4-Bar odo Pods so it loads factory resolution math
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            // Calibrates the gyro & zeroes out the starting position
            pinpoint.resetPosAndIMU();
        }

        // Pulls data from the hub and translates mm to inches
        @Override
        public void update() {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();

            currentPose = new Pose(
                    Localizer.mmToInches(p.getX(DistanceUnit.MM)),
                    Localizer.mmToInches(p.getY(DistanceUnit.MM)),
                    p.getHeading(AngleUnit.RADIANS)
            );
        }

        // Hands translated inches/radians position to the path follower
        @Override
        public Pose getPose() {
            return currentPose;
        }

        // Grabs speed v from the hub & translates mm/s to inches/s
        @Override
        public Velocity getVelocity() {
            Pose2D v = pinpoint.getVelocity();

            return new Velocity(
                    Localizer.mmToInches(v.getX(DistanceUnit.MM)),
                    Localizer.mmToInches(v.getY(DistanceUnit.MM)),
                    v.getHeading(AngleUnit.RADIANS)
            );
        }

        // Teleportation tool (lol so fancy), where is bot on the field
        @Override
        public void setPose(Pose p) {
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.MM,
                    Localizer.inchesToMm(p.x),
                    Localizer.inchesToMm(p.y),
                    AngleUnit.RADIANS,
                    p.heading
            ));
        }
    }

/// A manual math template for custom tracking layouts
/// Uses your Control Hub's main processor to calculate positions using basic trig
    public static abstract class TwoPodImuLocalizer implements Localizer {
//Protected variables so any baby class can read or change coordinates directly
        protected double x, y, heading;

//blueprint placeholder, return distance since last loop: [Forward, Strafe]
        protected abstract double[] readPodDeltasInches();

//blueprint placeholder, returns current gyro angle in c-clockwise radians
        protected abstract double readImuHeadingRadians();

//The custom trig integration engine
        @Override
        public void update() {
            double newHeading = readImuHeadingRadians();
            double[] d = readPodDeltasInches();

// Computes the midpoint angle of the turn to model motion as a smooth curved arc
            double mid = heading + LQRPathFollower.normalizeAngle(newHeading - heading) / 2.0;
            double c = Math.cos(mid);
            double s = Math.sin(mid);

// Matrix trig transformation: shifts robot-centric onto the field map
            x += d[0] * c - d[1] * s; // Update X pos
            y += d[0] * s + d[1] * c; // Update Y pos
            heading = newHeading;     // Save the new angle as the baseline for the next loop cycle
        }

// Standard position retriever
        @Override
        public Pose getPose() {
            return new Pose(x, y, heading);
        }

// Returns null b/c raw encoders don't track speed natively on hardware
        @Override
        public Velocity getVelocity() {
            return null;
        }

// Overwrites calculations w a manual starting position override
        @Override
        public void setPose(Pose p) {
            x = p.x;
            y = p.y;
            heading = p.heading;
        }
    }
}
