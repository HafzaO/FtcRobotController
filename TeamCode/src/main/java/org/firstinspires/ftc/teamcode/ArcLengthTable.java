
package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

public class ArcLengthTable {

    private final List<Double> tValues = new ArrayList<>(); // Storage for timeline percentage steps
    private final List<Double> lengths = new ArrayList<>(); // Storage for accumulated inch distances
    private final double totalLen;                          // Total length of the path in inches

    // Constructor: Breaks down a path shape into 100 tiny measuring segments
    public ArcLengthTable(HolonomicPath path) {
        int samples = 100;
        double accumulatedDist = 0.0;

        tValues.add(0.0);
        lengths.add(0.0);

        Vec2 lastPoint = path.pointAt(0.0);

        // Step across the curve timeline to measure linear distances
        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            Vec2 currentPoint = path.pointAt(t);

            // Measure straight-line distance gap from the last slice point
            accumulatedDist += lastPoint.dist(currentPoint);

            tValues.add(t);
            lengths.add(accumulatedDist);
            lastPoint = currentPoint;
        }

        totalLen = accumulatedDist;
    }

    public double totalLength() {
        return totalLen;
    }

    // Translates a target distance back into a precise time fraction 't'
    public double tAtArcLength(double s) {
        if (s <= 0.0) return 0.0;
        if (s >= totalLen) return 1.0;

        // Binary Search lookup engine: isolates which sample chunk contains distance 's'
        int lo = 0;
        int hi = lengths.size() - 1;

        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (lengths.get(mid) < s) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        // Linear interpolation math: fits the position precisely between the two nearest samples
        double s0 = lengths.get(lo);
        double s1 = lengths.get(hi);
        double t0 = tValues.get(lo);
        double t1 = tValues.get(hi);

        if (Math.abs(s1 - s0) < 1e-6) {
            return t0;
        }

        return t0 + (s - s0) * (t1 - t0) / (s1 - s0);
    }

    // Advanced search: finds the closest distance along a path shape to a real-world coordinate point
    public double closestArcLength(HolonomicPath path, Vec2 targetPoint) {
        double bestS = 0.0;
        double bestDist = Double.MAX_VALUE;
        int scans = 200;

        // Scan the entire trajectory table structure to find the closest match alignment
        for (int i = 0; i <= scans; i++) {
            double s = ((double) i / scans) * totalLen;
            double t = tAtArcLength(s);
            Vec2 pathPoint = path.pointAt(t);
            double d = pathPoint.dist(targetPoint);

            if (d < bestDist) {
                bestDist = d;
                bestS = s;
            }
        }
        return bestS;
    }
}

