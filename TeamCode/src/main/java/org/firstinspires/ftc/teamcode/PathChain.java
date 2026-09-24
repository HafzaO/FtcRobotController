package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//// Path Chain System
/// Links multiple separate lines and curves into one long, continuous autonomous path.

public class PathChain implements HolonomicPath {

    // Storage buckets for tracking path pieces
    private final List<HolonomicPath> segs = new ArrayList<>();   // The shape segments
    private final List<ArcLengthTable> tabs = new ArrayList<>();  // Distance lookup tables
    private final List<Double> starts = new ArrayList<>();        // Starting inch mark of each piece
    private double len; // Total length of the entire chain in inches

// Constructor: Takes one or more path shapes and strings them together
    public PathChain(HolonomicPath... parts) {
        if (parts.length == 0) {
            throw new IllegalArgumentException("a chain needs at least one segment");
        }
        for (HolonomicPath p : parts) {
            add(p);
        }
    }

// Appends a new path piece to the end of our current list
    private void add(HolonomicPath p) {
        ArcLengthTable t = new ArcLengthTable(p);
        starts.add(len);
        segs.add(p);
        tabs.add(t);
        len += t.totalLength();
    }
    public double totalLength() {
        return len;
    }
    public int segmentCount() {
        return segs.size();
    }
    public HolonomicPath segment(int i) {
        return segs.get(i);
    }

// Identifies which segment index a timeline fraction 't' falls inside
    public int segmentAt(double t) {
        double[] l = locate(t);
        if (l[0] == 0) {
            return 0;
        }
        return (int) l[0];
    }

// Master lookup engine: translates a global path time fraction 't' into a specific segment index
    private double[] locate(double t) {
        t = LinePath.clamp01(t);
        double s = t * len; // Target distance in inches along the chain
        int idx = segs.size() - 1;

// Find which path block owns this distance
        for (int i = 0; i < segs.size(); i++) {
            double start = starts.get(i);
            double end = start + tabs.get(i).totalLength();
            if (s <= end || i == segs.size() - 1) {
                idx = i;
                break;
            }
        }

// Translate global distance into a local percentage timer for that specific segment
        double localS = s - starts.get(idx);
        double localT = tabs.get(idx).tAtArcLength(localS);
        return new double[]{idx, localT};
    }
    @Override
    public Vec2 pointAt(double t) {
        double[] l = locate(t);
        return segs.get((int) l[0]).pointAt(l[1]);
    }
    @Override
    public Vec2 derivativeAt(double t) {
        double[] l = locate(t);
        return segs.get((int) l[0]).derivativeAt(l[1]);
    }
    @Override
    public Vec2 secondDerivativeAt(double t) {
        double[] l = locate(t);
        return segs.get((int) l[0]).secondDerivativeAt(l[1]);
    }
    @Override
    public double curvatureAt(double t) {
        double[] l = locate(t);
        return segs.get((int) l[0]).curvatureAt(l[1]);
    }

// Text formatting method that builds a string of combined path names for telem
    @Override
    public String name() {
        StringBuilder sb = new StringBuilder("Chain[");
        for (int i = 0; i < segs.size(); i++) {
            if (i > 0) {
                sb.append(" + ");
            }
            sb.append(segs.get(i).name());
        }
        return sb.append(']').toString();
    }

// Gathers and outputs all control guide points from every single attached path segment
    @Override
    public Vec2[] controlPoints() {
        List<Vec2> all = new ArrayList<>();
        for (HolonomicPath p : segs) {
            all.addAll(Arrays.asList(p.controlPoints()));
        }
        return all.toArray(new Vec2[0]);
    }

    // Allows path designer software to grab and reposition an anchor point dynamically
    @Override
    public void setControlPoint(int index, Vec2 p) {
        for (HolonomicPath seg : segs) {
            int n = seg.controlPoints().length;
            if (index < n) {
                seg.setControlPoint(index, p);
                rebuild();
                return;
            }
            index -= n;
        }
    }

// Clears out calibration parameters and recalculates path tables from scratch
    private void rebuild() {
        List<HolonomicPath> copy = new ArrayList<>(segs);
        segs.clear();
        tabs.clear();
        starts.clear();
        len = 0;
        for (HolonomicPath p : copy) {
            add(p);
        }
    }

/// Inner container class tracking structural seam parameters where two paths meet
    public static final class Joint {
        public final int index;
        public final double gap;   // Straight-line physical coordinate gap in inches
        public final double angle; // Tangent path angle breakdown in radians

        Joint(int index, double gap, double angle) {
            this.index = index;
            this.gap = gap;
            this.angle = angle;
        }

// Returns true if the two path segments blend into each other smoothly without sudden jumps
        public boolean isSmooth() {
            return gap < 0.25 && angle < Math.toRadians(5);
        }

        @Override
        public String toString() {
            return String.format("joint %d: gap %.2f in, tangent break %.1f deg%s", index, gap, Math.toDegrees(angle), isSmooth() ? "" : "   <-- NOT SMOOTH");
        }
    }

// Safety Checker: Loops through interior joints to detect broken or disjointed seams
    public List<Joint> validate() {
        List<Joint> out = new ArrayList<>();
        for (int i = 0; i < segs.size() - 1; i++) {
            HolonomicPath a = segs.get(i);
            HolonomicPath b = segs.get(i + 1);

// Measure spatial distance gap between segment transitions
            Vec2 endA = a.pointAt(1);
            Vec2 startB = b.pointAt(0);
            double d = endA.distanceTo(startB);

// Run trigonometric dot-product calculations to find direction kinking angles
            Vec2 ta = a.tangentAt(1);
            Vec2 tb = b.tangentAt(0);
            double dot = Math.max(-1, Math.min(1, ta.dot(tb)));

            out.add(new Joint(i, d, Math.acos(dot)));
        }
        return out;
    }

// Returns true if every interior path joint passes alignment threshold tolerances
    public boolean isSmooth() {
        for (Joint j : validate()) {
            if (!j.isSmooth()) {
                return false;
            }
        }
        return true;
    }
}
