package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Teleop with LQR driver assist.
 *
 * Translation stays raw so the stick feels connected. The LQR is used only for
 * heading lock and a hold-to-run score macro.
 *
 * Controls:
 *   sticks          field-centric drive
 *   Y / B / A / X   lock heading to 0 / 90 / 180 / 270 degrees
 *   left bumper     snap to nearest 90
 *   right bumper    release heading lock
 *   right trigger   HOLD to run the score macro, release to regain control
 */
@TeleOp(name = "LQR Driver Assist")
public class DriverAssistOpMode extends LinearOpMode {

    private static final double MAX_VELOCITY = 40.0;
    private static final double MAX_ACCEL    = 40.0;
    private static final double MAX_ANGULAR  = 3.0;

    private static final double SCORE_X = 48, SCORE_Y = 24, SCORE_HEADING = Math.PI / 2;
    private static final double STICK_DEADBAND = 0.05;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        Localizer localizer = createLocalizer();
        DriverAssist assist = DriverAssist.withDefaults(MAX_ANGULAR);
        LQRPathFollower macro = LQRPathFollower.withDefaults(MAX_VELOCITY, MAX_ACCEL, MAX_ANGULAR);

        boolean macroActive = false;
        ElapsedTime loopTimer = new ElapsedTime();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0) dt = 0.02;

            localizer.update();
            Localizer.Pose pose = localizer.getPose();

            if (gamepad1.y)  assist.lockHeading(0);
            if (gamepad1.b)  assist.lockHeading(Math.PI / 2);
            if (gamepad1.a)  assist.lockHeading(Math.PI);
            if (gamepad1.x)  assist.lockHeading(-Math.PI / 2);
            if (gamepad1.left_bumper)  assist.snapToNearest(pose.heading, 4);
            if (gamepad1.right_bumper) assist.release();

            boolean wantMacro = gamepad1.right_trigger > 0.5;

            // HOLD to run, never toggle. Releasing must return control instantly,
            // because this path is generated from live odometry with no field
            // collision checking of any kind.
            if (wantMacro && !macroActive) {
                macro.followPath(DriverAssist.scoreMacroPath(
                                pose.x, pose.y, pose.heading, SCORE_X, SCORE_Y, SCORE_HEADING),
                        pose.heading, SCORE_HEADING);
                macroActive = true;
            } else if (!wantMacro) {
                macroActive = false;
            }

            double[] powers;
            if (macroActive) {
                powers = macro.update(pose.x, pose.y, pose.heading, dt);
                telemetry.addLine("MACRO ACTIVE - release trigger for manual control");
            } else {
                double sx = deadband(gamepad1.left_stick_x);
                double sy = deadband(-gamepad1.left_stick_y);
                double st = deadband(gamepad1.right_stick_x);
                powers = assist.update(sx, sy, st, pose.heading);
            }

            setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

            telemetry.addData("Pose", pose.toString());
            telemetry.addData("Heading lock", assist.isLocked()
                    ? String.format("%.0f deg", Math.toDegrees(assist.lockedHeading())) : "off");
            telemetry.update();
        }

        setMotorPowers(0, 0, 0, 0);
    }

    private static double deadband(double v) {
        return Math.abs(v) < STICK_DEADBAND ? 0 : v;
    }

    /** STUB. See PathFollowerOpMode for why this throws instead of faking a pose. */
    private Localizer createLocalizer() {
        throw new UnsupportedOperationException("createLocalizer() is a stub.");
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl); frontRight.setPower(fr);
        backLeft.setPower(bl);  backRight.setPower(br);
    }
}
