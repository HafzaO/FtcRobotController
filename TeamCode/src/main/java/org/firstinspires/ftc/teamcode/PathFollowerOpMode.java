package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "LQR Path Follow")
public class PathFollowerOpMode extends LinearOpMode {

    // MEASURE THESE.
    private static final double MAX_VELOCITY = 40.0;    // in/s
    private static final double MAX_ACCEL = 40.0;       // in/s^2
    private static final double MAX_ANGULAR = 3.0;      // rad/s

    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double POSITION_TOLERANCE_IN = 1.0;
    private static final double ANGLE_TOLERANCE_RAD = 0.06;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private VoltageSensor voltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // TUNGUNGTODOOOOOO verify against wiring.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        LQRPathFollower follower = LQRPathFollower.withDefaults(MAX_VELOCITY, MAX_ACCEL, MAX_ANGULAR);

        // Paste a path here from the visualizer's P key
        HolonomicPath path = new CubicBezierPath(
                new Vec2(-48, -48), new Vec2(-10, 40),
                new Vec2(20, -40), new Vec2(48, 24));

        telemetry.addLine("Ready. Path length " + String.format("%.1f in", new ArcLengthTable(path).totalLength()));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        follower.followPath(path, /*startHeading*/ 0, /*endHeading*/ Math.PI / 2);

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0)
                dt = 0.02;

            double[] pose = readOdometry();
            double[] powers = follower.update(pose[0], pose[1], pose[2], dt);

            double voltage = voltageSensor.getVoltage();
            double scale = Math.min(1.0, voltage / NOMINAL_VOLTAGE);
            for (int i = 0; i < powers.length; i++)
                powers[i] *= scale;

            boolean valid = true;

            for (double p : powers)
                if (Double.isNaN(p) || Double.isInfinite(p)) valid = false;

            //i swr if this shows up all of yall r getting slimed out ;)
            if (!valid){
                setMotorPowers(0, 0, 0, 0);
                telemetry.addLine("WARNING: invalid power computed, forcing zero");
                telemetry.update();
                continue;
            }
            setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

            LQRPathFollower.Reference ref = follower.referenceAt(follower.elapsed());
            double posErr = Math.hypot(ref.position.x - pose[0], ref.position.y - pose[1]);
            double angErr = Math.abs(LQRPathFollower.normalizeAngle(ref.heading - pose[2]));
            if (follower.isFinished() && posErr < POSITION_TOLERANCE_IN && angErr < ANGLE_TOLERANCE_RAD) {
                break;
            }

            telemetry.addData("Progress", "%.2f / %.2f s", follower.elapsed(), follower.duration());
            telemetry.addData("Pose", "x=%.1f y=%.1f th=%.2f", pose[0], pose[1], pose[2]);
            telemetry.addData("Error", "%.2f in / %.3f rad", posErr, angErr);
            telemetry.addData("Battery", "%.2f V (scale %.2f)", voltage, scale);
            telemetry.addData("Loop", "%.0f Hz", 1.0 / dt);
            //i swr if this shows up all of yall r getting slimed out ;)
            if (follower.isSaturated())
                telemetry.addLine("saturated, integral frozen");
            telemetry.update();
        }

        setMotorPowers(0, 0, 0, 0);
    }

    private double[] readOdometry() {
        throw new UnsupportedOperationException(
                "readOdometry() is a stub. Wire up your pods plus IMU heading.");
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}