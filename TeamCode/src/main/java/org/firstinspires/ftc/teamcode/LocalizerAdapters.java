package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * A collection of hardware adapters that bridge real sensors to our Localizer rules.
 */
public final class LocalizerAdapters {

    // Stops anyone from accidentally creating a blank copy of this class
    private LocalizerAdapters() { }

    public static class PinpointLocalizer implements Localizer {
        private final GoBildaPinpointDriver pinpoint;
        private Pose currentPose = new Pose(0, 0, 0);

        // Connects & sets up the hardware when auto starts
        public PinpointLocalizer(HardwareMap hw, String name) {
            pinpoint = hw.get(GoBildaPinpointDriver.class, name);

            /// ESTIMATE CHANGE
            pinpoint.setOffsets(5.0, -50.0, DistanceUnit.MM);

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

        // Grabs speed vector from the hub & translates mm/s to inches/s
        @Override
        public Velocity getVelocity() {
            // Pull the raw X and Y speeds as individual numbers from the driver
            double xSpeedMm = pinpoint.getVelX(DistanceUnit.MM);
            double ySpeedMm = pinpoint.getVelY(DistanceUnit.MM);

            // Pull the angular velocity directly using the correct UnnormalizedAngleUnit type
            double spinSpeedRad = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

            // Translate the metric speeds to inches and return the full package!
            return new Velocity(
                    Localizer.mmToInches(xSpeedMm),
                    Localizer.mmToInches(ySpeedMm),
                    spinSpeedRad
            );
        }

        // Teleportation tool: sets the starting position configuration on the field map
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

    public static abstract class TwoPodImuLocalizer implements Localizer {
        // Protected variables so any child class can read or change coordinates directly
        protected double x, y, heading;

        // Blueprint placeholder: return distance since last loop: [Forward, Strafe]
        protected abstract double[] readPodDeltasInches();

        // Blueprint placeholder: returns current gyro angle in counter-clockwise radians
        protected abstract double readImuHeadingRadians();

        // The custom trigonometry integration engine
        @Override
        public void update() {
            double newHeading = readImuHeadingRadians();
            double[] d = readPodDeltasInches();

            // Computes the midpoint angle of the turn to model motion as a smooth curved arc
            double mid = heading + LQRPathFollower.normalizeAngle(newHeading - heading) / 2.0;
            double c = Math.cos(mid);
            double s = Math.sin(mid);

            // Matrix trig transformation: shifts robot-centric coordinates onto the global field map
            x += d[0] * c - d[1] * s; // Update X pos
            y += d[0] * s + d[1] * c; // Update Y pos
            heading = newHeading;     // Save the new angle as the baseline for the next loop cycle
        }

        // Standard position retriever
        @Override
        public Pose getPose() {
            return new Pose(x, y, heading);
        }

        // Returns null because raw encoders don't track speed natively on hardware
        @Override
        public Velocity getVelocity() {
            return null;
        }

        // Overwrites calculations with a manual starting position override
        @Override
        public void setPose(Pose p) {
            x = p.x;
            y = p.y;
            heading = p.heading;
        }
    }
}
