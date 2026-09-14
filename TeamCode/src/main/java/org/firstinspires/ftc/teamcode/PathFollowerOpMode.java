package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous: full stack. Localizer -> LQR -> ChassisDynamics -> motors.
 *
 * The OpMode does hardware I/O and nothing else. Every control decision lives
 * in classes with no SDK imports, which is why they can be unit tested on a
 * desktop JVM.
 *
 * BEFORE RUNNING:
 *  1. Pick a Localizer. The stub below throws rather than returning (0,0,0),
 *     because a plausible-looking zero pose sends the robot across the field.
 *  2. MEASURE Ks, Kv, Ka for ChassisDynamics. See that class for the procedure.
 *     Until then, pass null and the normalized mixer is used instead.
 *  3. Verify motor names and directions against your wiring.
 */
@Autonomous(name = "LQR Path Follow")
public class PathFollowerOpMode extends LinearOpMode {

    private static final double MAX_VELOCITY = 40.0;   // in/s  MEASURE
    private static final double MAX_ACCEL    = 40.0;   // in/s^2 MEASURE
    private static final double MAX_ANGULAR  = 3.0;    // rad/s MEASURE

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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        Localizer localizer = createLocalizer();

        // Chained auto, profiled end to end so the robot never stops at a
        // waypoint. In simulation this saved 0.7s over running the same two
        // segments as separate profiles.
        PathChain route = new PathChain(
                QuinticHermitePath.fromTangents(
                        new Vec2(-56, -56), new Vec2(90, 0),
                        new Vec2(0, -24),   new Vec2(90, 0)),
                new LinePath(new Vec2(0, -24), new Vec2(48, -24))
        );

        // Check the joints before driving. A tangent break costs roughly
        // speed x actuation-lag of corner rounding: about 0.8in at a 90 degree
        // joint in simulation, versus 0.06in at a smooth one.
        for (PathChain.Joint j : route.validate()) {
            telemetry.addLine(j.toString());
        }

        LQRPathFollower follower = LQRPathFollower.withDefaults(MAX_VELOCITY, MAX_ACCEL, MAX_ANGULAR);

        // Uncomment once Ks/Kv/Ka are MEASURED. Until then the normalized
        // mixer is the honest default.
        // follower.setChassisDynamics(ChassisDynamics.estimatedDefaults());

        // Uncomment after measuring your loop-to-motion delay. One or two loop
        // periods. Larger values reintroduce corner cutting.
        // follower.setLatencyCompensation(0.04);

        telemetry.addData("Route", "%s, %.1f in, smooth=%s",
                route.name(), route.totalLength(), route.isSmooth());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        localizer.update();
        follower.followPath(route, 0, Math.PI / 2);

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0) dt = 0.02;

            localizer.update();
            Localizer.Pose pose = localizer.getPose();
            double voltage = voltageSensor.getVoltage();

            double[] powers = follower.update(pose.x, pose.y, pose.heading, dt, voltage);

            boolean valid = true;
            for (double p : powers) if (Double.isNaN(p) || Double.isInfinite(p)) valid = false;
            if (!valid) {
                setMotorPowers(0, 0, 0, 0);
                telemetry.addLine("WARNING: invalid power computed, forcing zero");
                telemetry.update();
                continue;
            }
            setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

            LQRPathFollower.Reference ref = follower.referenceAt(follower.elapsed());
            double posErr = Math.hypot(ref.position.x - pose.x, ref.position.y - pose.y);
            double angErr = Math.abs(LQRPathFollower.normalizeAngle(ref.heading - pose.heading));

            // Arrival needs the profile finished AND the robot actually there.
            // Time alone is not arrival.
            if (follower.isFinished() && posErr < POSITION_TOLERANCE_IN && angErr < ANGLE_TOLERANCE_RAD) {
                break;
            }

            telemetry.addData("Segment", "%d / %d", follower.currentSegment() + 1, route.segmentCount());
            telemetry.addData("Progress", "%.2f / %.2f s", follower.elapsed(), follower.duration());
            telemetry.addData("Pose", pose.toString());
            telemetry.addData("Error", "%.2f in / %.3f rad", posErr, angErr);
            telemetry.addData("Battery", "%.2f V", voltage);
            telemetry.addData("Loop", "%.0f Hz", 1.0 / dt);
            if (follower.isSaturated()) telemetry.addLine("saturated, integral frozen");
            telemetry.update();
        }

        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * STUB. Return a real Localizer. See LocalizerAdapters for Pinpoint, OTOS
     * and two-pod-plus-IMU options.
     *
     * Throws rather than returning a zeroed simulated pose: a pose of (0,0,0)
     * looks plausible on telemetry and would drive the robot at full speed
     * toward a target it thinks is far away.
     */
    private Localizer createLocalizer() {
        throw new UnsupportedOperationException(
                "createLocalizer() is a stub. Return a PinpointLocalizer, OtosLocalizer, "
                        + "or your own TwoPodImuLocalizer subclass.");
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}