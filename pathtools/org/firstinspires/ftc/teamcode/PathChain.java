package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Several paths strung together and presented as ONE path.
 *
 * The design decision worth noting: PathChain implements HolonomicPath itself.
 * That means the follower, the arc-length table and the motion profile need no
 * changes at all to support chaining, and a chain can even contain another
 * chain. The alternative (teaching the follower about segment lists) would have
 * put sequencing logic in three places instead of one.
 *
 * A chain is profiled as a SINGLE trajectory end to end, so the robot does not
 * stop at every joint. That is the whole point: stopping at each waypoint is
 * what makes an auto slow.
 *
 * CONTINUITY: validate() reports joints that are not smooth. A position gap
 * (C0) teleports the reference and the robot will lurch. A tangent-direction
 * break (C1) demands an instantaneous change of travel direction, which no
 * drivetrain can produce, so the robot rounds it off and leaves the path. These
 * are reported rather than auto-corrected, because silently "fixing" someone's
 * geometry hides a design mistake they need to see.
 */
public class PathChain implements HolonomicPath {

    private final List<HolonomicPath> segments = new ArrayList<>();
    private final List<ArcLengthTable> tables = new ArrayList<>();
    private final List<Double> cumulativeStart = new ArrayList<>();
    private double totalLength;

    public PathChain(HolonomicPath... parts) {
        if (parts.length == 0) throw new IllegalArgumentException("a chain needs at least one segment");
        for (HolonomicPath p : parts) add(p);
    }

    private void add(HolonomicPath p) {
        ArcLengthTable t = new ArcLengthTable(p);
        cumulativeStart.add(totalLength);
        segments.add(p);
        tables.add(t);
        totalLength += t.totalLength();
    }

    public double totalLength() { return totalLength; }
    public int segmentCount()   { return segments.size(); }
    public HolonomicPath segment(int i) { return segments.get(i); }

    /** Which segment index a global parameter t lands in. Useful for telemetry. */
    public int segmentAt(double t) { return locate(t)[0] == 0 ? 0 : (int) locate(t)[0]; }

    /**
     * Maps global t in [0,1] to {segmentIndex, localT}.
     * Global t is distributed by ARC LENGTH, not by segment count, so a 60 inch
     * segment occupies six times the parameter span of a 10 inch one. Splitting
     * t evenly per segment instead would make the robot crawl through long
     * segments and sprint through short ones.
     */
    private double[] locate(double t) {
        t = LinePath.clamp01(t);
        double s = t * totalLength;
        int idx = segments.size() - 1;
        for (int i = 0; i < segments.size(); i++) {
            double start = cumulativeStart.get(i);
            double end = start + tables.get(i).totalLength();
            if (s <= end || i == segments.size() - 1) { idx = i; break; }
        }
        double localS = s - cumulativeStart.get(idx);
        double localT = tables.get(idx).tAtArcLength(localS);
        return new double[]{idx, localT};
    }

    @Override public Vec2 pointAt(double t) {
        double[] l = locate(t);
        return segments.get((int) l[0]).pointAt(l[1]);
    }

    /**
     * NOTE: returns the SEGMENT-LOCAL derivative, not one rescaled to global t.
     * Only direction is consumed downstream (tangentAt normalises it, and
     * curvatureAt is overridden below to delegate), so the magnitude never
     * matters. Rescaling would need dLocalT/dGlobalT, which is discontinuous at
     * every joint and would produce misleading numbers.
     */
    @Override public Vec2 derivativeAt(double t) {
        double[] l = locate(t);
        return segments.get((int) l[0]).derivativeAt(l[1]);
    }

    @Override public Vec2 secondDerivativeAt(double t) {
        double[] l = locate(t);
        return segments.get((int) l[0]).secondDerivativeAt(l[1]);
    }

    /** Delegated so curvature stays exact despite the local parameterization. */
    @Override public double curvatureAt(double t) {
        double[] l = locate(t);
        return segments.get((int) l[0]).curvatureAt(l[1]);
    }

    @Override public String name() {
        StringBuilder sb = new StringBuilder("Chain[");
        for (int i = 0; i < segments.size(); i++) {
            if (i > 0) sb.append(" + ");
            sb.append(segments.get(i).name());
        }
        return sb.append(']').toString();
    }

    @Override public Vec2[] controlPoints() {
        List<Vec2> all = new ArrayList<>();
        for (HolonomicPath p : segments) all.addAll(Arrays.asList(p.controlPoints()));
        return all.toArray(new Vec2[0]);
    }

    @Override public void setControlPoint(int index, Vec2 p) {
        for (HolonomicPath seg : segments) {
            int n = seg.controlPoints().length;
            if (index < n) { seg.setControlPoint(index, p); rebuild(); return; }
            index -= n;
        }
    }

    private void rebuild() {
        List<HolonomicPath> copy = new ArrayList<>(segments);
        segments.clear(); tables.clear(); cumulativeStart.clear(); totalLength = 0;
        for (HolonomicPath p : copy) add(p);
    }

    // ---------------- continuity checking ----------------

    public static final class Joint {
        public final int index;
        public final double positionGapInches;
        public final double tangentBreakRadians;
        Joint(int index, double gap, double breakRad) {
            this.index = index; this.positionGapInches = gap; this.tangentBreakRadians = breakRad;
        }
        public boolean isSmooth() {
            return positionGapInches < 0.25 && tangentBreakRadians < Math.toRadians(5);
        }
        @Override public String toString() {
            return String.format("joint %d: gap %.2f in, tangent break %.1f deg%s",
                    index, positionGapInches, Math.toDegrees(tangentBreakRadians),
                    isSmooth() ? "" : "   <-- NOT SMOOTH");
        }
    }

    /** One entry per interior joint. Empty for a single-segment chain. */
    public List<Joint> validate() {
        List<Joint> out = new ArrayList<>();
        for (int i = 0; i < segments.size() - 1; i++) {
            HolonomicPath a = segments.get(i), b = segments.get(i + 1);
            Vec2 endA = a.pointAt(1), startB = b.pointAt(0);
            double gap = endA.distanceTo(startB);

            Vec2 ta = a.tangentAt(1), tb = b.tangentAt(0);
            double dot = Math.max(-1, Math.min(1, ta.dot(tb)));
            out.add(new Joint(i, gap, Math.acos(dot)));
        }
        return out;
    }

    /** Convenience: true when every joint is within tolerance. */
    public boolean isSmooth() {
        for (Joint j : validate()) if (!j.isSmooth()) return false;
        return true;
    }
}
