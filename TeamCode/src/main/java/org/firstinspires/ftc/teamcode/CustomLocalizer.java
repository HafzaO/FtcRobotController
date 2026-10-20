package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.LocalizerAdapters;

public class CustomLocalizer extends LocalizerAdapters.TwoPodImuLocalizer {
    private final DcMotorEx parallelE;
    private final DcMotorEx perpE;
    private final IMU imu;
    private int lastParallelPos = 0;
    private int lastPerpPos = 0;

    private static final double INCHES_PER_TICK = (2.0 * Math.PI * 0.748) / 8192.0;
// Adjust to odo pods (i think this is right for blue got it off of gb website)
// ticks per min = encoder cpr/wheel D in mm*pi

    public CustomLocalizer(HardwareMap hw, String parallelName, String perpName, String imuName) {
        parallelE = hw.get(DcMotorEx.class, parallelName);
        perpE = hw.get(DcMotorEx.class, perpName);
        imu = hw.get(IMU.class, imuName);
    }
// setup block that runs once when your auto starts
// It searches through bot's config map to sync code vars to the actual ports
// you named on your driver station app

    @Override
    protected double[] readPodDeltasInches() {
        int currentParallel = parallelE.getCurrentPosition();
        int currentPerp = perpE.getCurrentPosition();

        double deltaForward = (currentParallel - lastParallelPos) * INCHES_PER_TICK;
        double deltaStrafe = (currentPerp - lastPerpPos) * INCHES_PER_TICK;

        lastParallelPos = currentParallel;
        lastPerpPos = currentPerp;

        return new double[] {
                deltaForward, deltaStrafe
        };
    }

    @Override
    protected double readImuHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}