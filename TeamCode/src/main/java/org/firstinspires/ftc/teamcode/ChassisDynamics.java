package org.firstinspires.ftc.teamcode;

public class ChassisDynamics {
    // This is the blueprint for robots weight, size + motor tuning numbers
    private double ks; // ks = static friction (raw voltage needed to overcome friction + get wheels to move)
    private double kv; // kv = voltage for constant velocity
    private double ka; // ka = voltage to break inertia & acc
    private double nominalV; // target voltage to tune for 11V
    private double trackRadius; // (halfTrackWidth + halfWheelBase), inches, dist from absolute center to corners to calculate turning levers (get enough torque)
    private double maxV; // 13.5V

    private static final double MAX_C_BOOST = 1.25;
    // Safety compensation cap so battery doesn't drop too low during any time of the match

    public double massKg;
    public double momentOfInertia;
    // Weight distribution not rly altering motor power js to check info

    public ChassisDynamics(double iKs, double iKv, double iKa, double trackWinI, double wheelBinI, double iNominalV, double iMaxV, double iMassKg, double iMOfIn) {
        if (iKv <= 0) {
            throw new IllegalArgumentException("kv must be > 0 (measure it don't guess)");
        }
        if (iKa < 0) {
            throw new IllegalArgumentException("ka must be >= 0");
        }
        if (iKs < 0) {
            throw new IllegalArgumentException("ks must be >= 0");
        }
        if (trackWinI <= 0 || wheelBinI <= 0) {
            throw new IllegalArgumentException("track width and wheel base must be > 0");
        }
        ks = iKs;
        kv = iKv;
        ka = iKa;

        nominalV = iNominalV;
        maxV = iMaxV;
        massKg = iMassKg;
        momentOfInertia = iMOfIn;

        double tD = trackWinI + wheelBinI;
        trackRadius = tD / 2.0;
    }

    public static ChassisDynamics estimatedDefaults() {
        // Safe generic values for a typical 18x18-inch FTC robot running goBILDA 312 RPM motors change thse tho
        return new ChassisDynamics(0.08, 0.0155, 0.0022, 14.0, 14.0, 12.0, 12.0, 14.0, 0.45);
    }

    public double[] wheelVs(double vx, double vy, double omega) {
        // Takes your desired overall robot movement velocities (vx = forward, vy = strafe, omega = rotation speed) and maps them to find how fast each wheel need to spin
        double r = trackRadius;
        return new double[]{
                vx + vy + r * omega,
                vx - vy - r * omega,
                vx - vy + r * omega,
                vx + vy - r * omega
        };
    }

    public double[] toMotorPowers(double vx, double vy, double omega, double ax, double ay, double alpha, double bV) {
        // Calculate the movement velocities and accelerations per wheel using kinematic equations
        double[] targetV = wheelVs(vx, vy, omega);
        double[] targetA = wheelVs(ax, ay, alpha);

        // Fix: We create a distinct 'volts' array so we do not overwrite our velocity array metrics!
        double[] volts = new double[4];
        for (int i = 0; i < 4; i++) {
            // Runs V = k_s * sign(v) + (k_v * v) + (k_a * a) to compute raw voltage commands
            volts[i] = ks * Math.signum(targetV[i]) + kv * targetV[i] + ka * targetA[i];

            // If a wheel is supposed to be still, force voltage to zero so ks doesn't make the motors jitter when breaking or parking
            if (Math.abs(targetV[i]) < 1e-6 && Math.abs(targetA[i]) < 1e-6) {
                volts[i] = 0;
            }
        }

        // Proportional Voltage Clamping!!
        // If the math calculates that a wheel needs more voltage than your battery can physically give (asking for 14V when you only have 12),
        // it scales down all 4 wheels together by the exact same ratio (k)
        // This makes sure the bot drives in the exact direction instead of steering off
        double peakVolts = 0;
        for (double x : volts) {
            peakVolts = Math.max(peakVolts, Math.abs(x));
        }

        if (peakVolts > maxV) {
            double k = maxV / peakVolts;
            for (int i = 0; i < 4; i++) {
                volts[i] *= k;
            }
        }

        // Battery Voltage Compensation.
        // If battery is fresh (13.5V), it scales motor inputs down slightly
        // If battery drops (11V), it scales inputs up to match
        // This makes auto behave the same no matter battery V
        double effectiveV = bV;
        if (effectiveV < 1e-3) {
            effectiveV = nominalV; // bad reading, fail safe
        }

        double compensationFactor = nominalV / effectiveV;
        if (compensationFactor > MAX_C_BOOST) {
            compensationFactor = MAX_C_BOOST;
        }

        // Converts the raw target V into a final scale ranging from -1.0 to 1.0 according to the FTC Hardware map
        // then it clamps them to not get errors and passes those out to the driving motors
        double[] pwr = new double[4];
        for (int i = 0; i < 4; i++) {
            double normalPower = (volts[i] / nominalV) * compensationFactor;
            pwr[i] = Math.max(-1, Math.min(1, normalPower));
        }
        return pwr;
    }

    // Auto path asking if this move is physically possible without exceeding our battery limits
    // and then It returns true or false
    public boolean isFeasible(double vx, double vy, double omega, double ax, double ay, double alpha) {
        double[] v = wheelVs(vx, vy, omega);
        double[] a = wheelVs(ax, ay, alpha);

        for (int i = 0; i < 4; i++) {
            double va = ks * Math.signum(v[i]) + kv * v[i] + ka * a[i];
            if (Math.abs(va) > maxV) {
                return false;
            }
        }
        return true;
    }

    // Calculates top velocity of bot that is manageable on a straight line based on the voltage budget we got
    public double maxAchievableVelocity() {
        return (maxV - ks) / kv;
    }

    // Allows other pathing files to read the calculated trackRadius value
    public double trackRadius() {
        return trackRadius;
    }
}
