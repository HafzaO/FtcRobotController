package org.firstinspipackage org.firstinspires.ftc.teamcode;

 /// The master blueprint rules for tracking where the robot is on the field
 /// Every sensor system (Pinpoint, OTOS, etc.) must follow these exact rules:
 ///     - Distance must be converted into INCHES (0 is the dead center of the field)
 ///     - Angles must be RADS

public interface Localizer {

    // Reads the tracking hardware once per loop cycle
    void update();

    // Returns the latest stored position and angle of the robot
    Pose getPose();

    // Returns the current driving and spinning speeds (returns null if unsupported)
    Velocity getVelocity();

    // Manually forces the robot's coordinate map to a specific spot (like at match start)
    void setPose(Pose p);

    // Conversion factor: 25.4 millimeters in 1 inch
    double MM_IN = 25.4;

    // mm to inches
    static double mmToInches(double mm) {
        return mm / MM_IN;
    }

    // inches to mm
    static double inchesToMm(double in) {
        return in * MM_IN;
    }
}


 ///A simple container data box that holds the robot's position on the field grid.
class Pose {
    public final double x;       // Horizontal coordinate on the field map
    public final double y;       // Vertical coordinate on the field map
    public final double heading; // Spin angle in radians

    // The builder that sets up our position numbers
    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Formats the coordinates nicely as readable text for the driver's phone screen
    @Override
    public String toString() {
        double degrees = Math.toDegrees(heading);
        return String.format("(%.1f, %.1f, %.1f deg)", x, y, degrees);
    }
}

 ///A simple container data box that holds how fast the robot is currently moving.
class Velocity {
    public final double vx;    // Speed driving forward/backward (inches per second)
    public final double vy;    // Speed strafing sideways (inches per second)
    public final double omega; // Speed spinning around (radians per second)

// The builder that sets up our speed numbers
    public Velocity(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }
}


 ///A completely virtual tracker. It doesn't connect to real sensors. It is used to test autonomous code inside a computer simulator or benchmarking tool.
class SimulatedLocalizer implements Localizer {
    private Pose p; // Storage box for the simulated position
    private Velocity v = new Velocity(0, 0, 0); // Storage box for the simulated speed

// Constructor: sets where the virtual robot starts on the map
    SimulatedLocalizer(double x, double y, double heading) {
        p = new Pose(x, y, heading);
    }

    @Override
    public void update() {
        // A virtual robot has no sensors to read, so this loop stays empty!
    }
    @Override
    public Pose getPose() {
        return p;
    }
    @Override
    public Velocity getVelocity() {
        return v;
    }
    @Override
    public void setPose(Pose pose) {
        p = pose;
    }

/// A testing tool! lets progs manually push fake loc and speed numbers directly into the simulator to test how paths react
    public void inject(double x, double y, double heading, double vx, double vy, double omega) {
        p = new Pose(x, y, heading);
        v = new Velocity(vx, vy, omega);
    }
}
