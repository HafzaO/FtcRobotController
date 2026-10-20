package org.firstinspires.ftc.teamcode;

public class TrapezoidalProfile {

    private final double totalDistance;
    private final double maxVelocity;
    private final double maxAcceleration;

    private final double peakVelocity;
    private final double accelDistance;
    private final double cruiseDistance;

    private final double accelTime;
    private final double cruiseTime;
    private final double totalTime;

    public TrapezoidalProfile(double totalDistance, double maxVelocity, double maxAcceleration) {
        if (totalDistance < 0) {
            throw new IllegalArgumentException("totalDistance must be >= 0");
        }
        if (maxVelocity <= 0) {
            throw new IllegalArgumentException("maxVelocity must be > 0");
        }
        if (maxAcceleration <= 0) {
            throw new IllegalArgumentException("maxAcceleration must be > 0");
        }

        this.totalDistance = totalDistance;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        if (totalDistance < 1e-9) {
            peakVelocity = 0;
            accelDistance = 0;
            cruiseDistance = 0;
            accelTime = 0;
            cruiseTime = 0;
            totalTime = 0;
        } else {
            double distanceNeededToReachMaxVelocity = (maxVelocity * maxVelocity) / (2 * maxAcceleration);
            double halfDistance = totalDistance / 2;

            if (distanceNeededToReachMaxVelocity <= halfDistance) {
                peakVelocity = maxVelocity;
                accelDistance = distanceNeededToReachMaxVelocity;
            } else {
                peakVelocity = Math.sqrt(maxAcceleration * totalDistance);
                accelDistance = halfDistance;
            }

            accelTime = peakVelocity / maxAcceleration;
            cruiseDistance = totalDistance - (2 * accelDistance);
            cruiseTime = cruiseDistance / peakVelocity;
            totalTime = (2 * accelTime) + cruiseTime;
        }
    }

    public double duration() {
        return totalTime;
    }

    public double posAt(double t) {
        if (totalDistance < 1e-9) {
            return 0;
        }
        if (t <= 0) {
            return 0;
        }
        if (t >= totalTime) {
            return totalDistance;
        }

        if (t < accelTime) {
            return 0.5 * maxAcceleration * t * t;
        }

        double cruiseEndTime = accelTime + cruiseTime;
        if (t < cruiseEndTime) {
            double timeInCruise = t - accelTime;
            return accelDistance + (peakVelocity * timeInCruise);
        }

        double timeInDecel = t - cruiseEndTime;
        double distanceAtCruiseEnd = accelDistance + cruiseDistance;
        double decelDistance = (peakVelocity * timeInDecel) - (0.5 * maxAcceleration * timeInDecel * timeInDecel);
        return distanceAtCruiseEnd + decelDistance;
    }

    public double vAt(double t) {
        if (totalDistance < 1e-9) {
            return 0;
        }
        if (t <= 0) {
            return 0;
        }
        if (t >= totalTime) {
            return 0;
        }

        if (t < accelTime) {
            return maxAcceleration * t;
        }

        double cruiseEndTime = accelTime + cruiseTime;
        if (t < cruiseEndTime) {
            return peakVelocity;
        }

        double timeInDecel = t - cruiseEndTime;
        return peakVelocity - (maxAcceleration * timeInDecel);
    }

    public double timeAtPosition(double s) {
        if (totalDistance < 1e-9) {
            return 0;
        }
        if (s <= 0) {
            return 0;
        }
        if (s >= totalDistance) {
            return totalTime;
        }

        double lowTime = 0;
        double highTime = totalTime;

        for (int i = 0; i < 100; i++) {
            double midTime = (lowTime + highTime) / 2;
            double midPosition = posAt(midTime);
            if (midPosition < s) {
                lowTime = midTime;
            } else {
                highTime = midTime;
            }
        }

        return (lowTime + highTime) / 2;
    }
}
