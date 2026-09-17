package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public final class LocalizerAdapters {

    private LocalizerAdapters() { }

    /*
     *getPosition() reports MILLIMETRES
     * putting it in inches-based stack does not throw, it just drives the
     * robot 25.4x too far + The conversion is the only place that matters
     *
     * Setup order that actually works:
     *   pinpoint.setOffsets(xOffsetMm, yOffsetMm);
     *   pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
     *   pinpoint.resetPosAndIMU();   // keep robot still
     *   sleep(300);                  // wait for IMU to calibrate
     * Do not put the Pinpoint on I2C port 0
     *
*/
    public static class PinpointLocalizer implements Localizer {
        private final GoBildaPinpointDriver pinpoint;
        private Pose pose = new Pose(0, 0, 0);

        public PinpointLocalizer(HardwareMap hw, String name) {
            pinpoint = hw.get(GoBildaPinpointDriver.class, name);
            pinpoint.setOffsets(15.0, -50.0); // Replace 15.0 and -50.0 with your actual measured X and Y in mm
            // saves us a bunch of math since it tells us how many "encoder ticks" equal 1 millimeter of physical floor travel.
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.resetPosAndIMU();
        }

        @Override public void update() {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();
            pose = new Pose(
                Localizer.mmToInches(p.getX(DistanceUnit.MM)),
                Localizer.mmToInches(p.getY(DistanceUnit.MM)),
                p.getHeading(AngleUnit.RADIANS));
        }

        @Override public Pose getPose() {
            return pose;
        }

        //calc & trig for this needs us to do in rads, degs will cause overcorrection
        @Override public Velocity getVelocity() {
            Pose2D v = pinpoint.getVelocity();
            return new Velocity(
                Localizer.mmToInches(v.getX(DistanceUnit.MM)),
                Localizer.mmToInches(v.getY(DistanceUnit.MM)),
                v.getHeading(AngleUnit.RADIANS));
        }
//tells the Pinpoint computer where to put the robot on the field at the start of auto
        @Override public void setPose(Pose p) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.MM,
                Localizer.inchesToMm(p.x), Localizer.inchesToMm(p.y),
                AngleUnit.RADIANS, p.heading));
        }
    }


    /**
     * this block uses the Control Hub's processor to calculate how far the robot drove
     * by running standard trigonometry (Math.cos and Math.sin) every time the robot moves
     */
    public static abstract class TwoPodImuLocalizer implements Localizer {
        // I used protect so any class can extend this template to read or change these values
        protected double x, y, heading;

        protected abstract double[] readPodDeltasInches();
        //placeholder (this why it is abstract) need to code in how many inces the wheels have rolled since the last loop check
        //list the values: [fwdDistance, strafeDistance]
        protected abstract double readImuHeadingRadians();

        @Override public void update() {
            double newHeading = readImuHeadingRadians();
            //asks how many inches the wheels traveled since the last loop check & store to then list in d
            double[] d = readPodDeltasInches();

            //calcs midpoint angle of turn, treats fwrd drive+spin as a straight line
            double mid = heading + LQRPathFollower.normalizeAngle(newHeading - heading) / 2.0;
            // sin + cos of heading arch
            double c = Math.cos(mid);
            double s = Math.sin(mid);
            //updates X pos, first sclaes fwd movement as d[0], strafe movement as d[1] + trig vecotr to find now horizontal coordinates
            x += d[0] * c - d[1] * s;
            y += d[0] * s + d[1] * c;
            heading = newHeading;
        }

        //Where is the robot right now?
        @Override public Pose getPose() {
            return new Pose(x, y, heading);
        }
        @Override public Velocity getVelocity() {
            return null;
        }

        //speed via simple time math
        @Override public void setPose(Pose p) {
            x = p.x; y = p.y; heading = p.heading;
        }
    }
}
