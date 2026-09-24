package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/// Controls:
/// Left Stick: Move the robot
/// Right Stick: Manual turning (Only works when heading lock is turned off)
/// Y / B / A / X: Lock facing direction to 0 / 90 / 180 / 270 degrees
/// Left Bumper: Snap to the nearest 90-degree angle
/// Right Bumper: Turn off the heading lock (Returns full control to right stick)
/// Right Trigger: HOLD to automatically score (Drives straight toward target)

@TeleOp(name = "LQR Driver Assist")
public class DriverAssistOpMode extends LinearOpMode {

    //basically safety params + account for stick driffftt
    private static final double MAX_V = 40.0;
    private static final double MAX_A    = 40.0;
    private static final double MAX_ANG  = 3.0;

    private static final double SCORE_X = 48, SCORE_Y = 24, SCORE_HEADING = Math.PI / 2;
    private static final double STICK_D = 0.05;

    private DcMotor fL, fR, bL, bR;

    @Override
    public void runOpMode() throws InterruptedException {
        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL   = hardwareMap.get(DcMotor.class, "bL");
        bR  = hardwareMap.get(DcMotor.class, "bR");
        fL.setDirection(DcMotor.Direction.FORWARD); ///check
        bL.setDirection(DcMotor.Direction.FORWARD); ///check

        //assist manages heading lock & macro works on path curves when the scoring trigger is pressed
        Localizer localizer = createLocalizer();
        DriverAssist assist = DriverAssist.withDefaults(MAX_ANG);
        LQRPathFollower macro = LQRPathFollower.withDefaults(MAX_V, MAX_A, MAX_ANG);

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

//lock the robot to preset field angles
            if (gamepad1.y)
                assist.lockHeading(0);
            if (gamepad1.b)
                assist.lockHeading(Math.PI / 2);
            if (gamepad1.a)
                assist.lockHeading(Math.PI);
            if (gamepad1.x)
                assist.lockHeading(-Math.PI / 2);

//left bumper snaps to nearest 90 quad, right resets to manual
            if (gamepad1.left_bumper)
                assist.snapToNearest(pose.heading, 4);
            if (gamepad1.right_bumper)
                assist.release();

//checks for right trigger, smooths path from bot location + trigger = imediate change
            boolean wantMacro = gamepad1.right_trigger > 0.5;
            if (wantMacro && !macroActive) {
                macro.followPath(DriverAssist.scoreMacroPath(
                                pose.x, pose.y, pose.heading, SCORE_X, SCORE_Y, SCORE_HEADING),
                        pose.heading, SCORE_HEADING);
                macroActive = true;
            }
            else if (!wantMacro) {
                macroActive = false;
            }

            double[] powers;
            if (macroActive) {
                powers = macro.update(pose.x, pose.y, pose.heading, dt);
                telemetry.addLine("MACRO ACTIVE - release trigger for manual control");
            } else {
                // Manual inputs read directly
                double sx = dead(gamepad1.left_stick_x);
                double sy = dead(-gamepad1.left_stick_y);
                double st = dead(gamepad1.right_stick_x);
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

    private static double dead(double v) {
//if not at 0 perfeectly prefents motor drift
        return Math.abs(v) < STICK_D ? 0 : v;
    }

    private Localizer createLocalizer() {
        // Stub implementation placeholder to prevent compile crashes
        return new Localizer() {
            private Pose p = new Pose(0,0,0);
            @Override public void update() {

            }
            @Override public Pose getPose() {
                return p;
            }
            @Override public void setPose(Pose pose) {
                this.p = pose;
            }
            @Override public V getV() {
                return null;
            }
        };
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        fL.setPower(fl); fR.setPower(fr);
        bL.setPower(bl);  bR.setPower(br);
    }
}