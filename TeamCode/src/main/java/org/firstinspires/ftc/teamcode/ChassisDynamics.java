package org.firstinspires.ftc.teamcode;

public class ChassisDynamics {
// this is the blueprint for robots weight, size + motor tuning numbers
    private final double ks; //  ks = static friction (raw voltage needed to overcome friction + get wheels to move)
    private final double kv; //  kv = voltage for constant velocity
    private final double ka; //  ka = voltage to break inertia & acc
    private final double nominalV; // target voltage to tune for 11V
    private final double trackRadius;   // (halfTrackWidth + halfWheelBase), inches, dist from absolute center to corners to calculate turing levers (get enough torque)
    private final double maxV; // 13.5V

    private static final double MAX_C_BOOST = 1.25;
    //safety compensation cap so batery doesnt drop too low during any time of the match

    public final double massKg;
    public final double momentOfInertia;
// weight distributon not rly altering motor power js to check info

public ChassisDynamics(double iKs, double iKv, double iKa, double trackWinI, double wheelinI, double iNominalV, double iMaxV, double iMassKg, double iMOfIn) {
    if (iKv <= 0) {
        System.err.println("ERROR: kv must be > 0 (measure it don't guess fattie)");
        return;
    }
    if (iKa < 0) {
        System.err.println("ERROR: ka must be >= 0");
        return;
    }
    if (iKs < 0) {
        System.err.println("ERROR: ks must be >= 0");
        return;
    }
    // Check track width and wheel base
    if (trackWinI <= 0 || wheelBinI <= 0) {
        System.err.println("ERROR: track width and wheel base aint > 0");
        return;
    }
    ks = iKs;
    kv = iKv;
    ka = iKa;

    nominalV = iNominalV;
    maxV = iMaxV;
    massKg = iMassKg;
    momentOfInertia = iMOfIn;

    double tD = trackWidthInches + wheelBaseInches;
    trackRadius = tD / 2.0;
    }

    public static ChassisDynamics estimatedDefaults() {
 //safe generic values for a typical 18x18-inch FTC robot running goBILDA 312 RPM motors change thse tho
        return new ChassisDynamics(0.08, 0.0155, 0.0022, 14.0, 14.0, 12.0, 12.0, 14.0, 0.45);
    }

    public double[] wheelVs(double vx, double vy, double omega) {
 //takes your desired overall robot movement velocities (vx = forward, vy = strafe, omega = rotation speed) and maps them to find how fast each wheel need to spin
        double r = trackRadius;
        return new double[]{
                vx + vy + r * omega,
                vx - vy - r * omega,
                vx - vy + r * omega,
                vx + vy - r * omega
        };
    }

    public double[] toMotorPowers(double vx, double vy, double omega, double ax, double ay, double alpha, double bV) {
 //Calculate the movement vectors tgth for standard Mecanum dt i found online, returning a 4-item list w target speeds for [FL, FR, BL, BR]

        double[] v = wheelVs(vx, vy, omega); //target velocity per wheel
        double[] a = wheelVs(ax, ay, alpha); //target acc per wheel

 //Runs V = k_s + (k_v dot v) + (k_a dot a) It calculates the raw voltage for each motor to match the paths
        double[] v = new double[4];
        for (int i = 0; i < 4; i++) {
            v[i] = ks * Math.signum(v[i]) + kv * v[i] + ka * a[i];

//If a wheel is supposed to be still, force voltage to zero so ks doesn't make the motors jitter when breakiig or parking
            if (Math.abs(v[i]) < 1e-6 && Math.abs(a[i]) < 1e-6)
                v[i] = 0;
        }

//Proportional Voltage Clamping!!
// If the math calculates that a wheel needs more voltage than your battery can physically give (asking for 14V when you only have 12),
// it scales down all 4 wheels together by the exact same ratio (k)
// This maeks sure the bot drives in the exact direction instead of steering off
        double p = 0;
        for (double x : v) p = Math.max(p, Math.abs(x));

        if (p > maxV) {
            double k = maxV / p;
            for (int i = 0; i < 4; i++)
                v[i] *= k;
        }

//Battery Voltage Compensation.
// If battery is fresh (13.5V), it scales motor inputs down slightly
// If battery drops (11V), it scales inputs up to match
// This makes auto behave the same no matter battery V

        double e = bV;
        if (e < 1e-3) e = nominalV;   // bad reading, fail safe
        double c = nominalV / e;
        if (c > MAX_C_BOOST) c = MAX_C_BOOST;

// Converts the raw target V into a final scale ranging from -1.0 to 1.0 according to the FTC Hardware map
// then it clamps them to not get errors and passes those out to the driving motors
        double[] pwr = new double[4];
        for (int i = 0; i < 4; i++) {
            double p = (v[i] / nominalV) * c;
            pwr[i] = Math.max(-1, Math.min(1, p));
        }
        return pwr;
    }

// Auto path asking if this move is physically possible without exceeding our battery limits
// and then It returns true or false
    public boolean isFeasible(double vx, double vy, double omega, double ax, double ay, double alpha) {

        double[] v = wheelVs(vx, vy, omega);
        double[] a = wheelVs(ax, ay, alpha);

        for (int i = 0; i < 4; i++) {
            double v = ks * Math.signum(v[i]) + kv * v[i] + ka * a[i];

            if (Math.abs(v) > maxV)
                return false;
        }

        return true;
    }

// Calculates topp velocity of bot that is managable on a straight line based on the voltage budget we got
    public double maxAchievableVelocity() {
        return (maxV - ks) / kv;
    }

// allows other pathing files to read the calculated trackRadius value
    public double trackRadius() {
        return trackRadius;
    }
}
