package org.firstinspires.ftc.teamcode;

public interface HolonomicPath {

    /** Position at parameter t in [0,1]. */
    Vec2 pointAt(double t);

    /** First derivative dP/dt. Direction is the tangent; magnitude is arbitrary. */
    Vec2 derivativeAt(double t);

    /** Second derivative d2P/dt2. Needed for curvature. */
    Vec2 secondDerivativeAt(double t);

    /** Human-readable name, used by the visualizer. */
    String name();

    /** Control/definition points, for the visualizer to draw and drag. */
    Vec2[] controlPoints();

    /** Replace a control point (visualizer dragging). Index is into controlPoints(). */
    void setControlPoint(int index, Vec2 p);

    /**
     * Signed curvature at t. kappa = (x'y'' - y'x'') / |P'|^3
     * Units are 1/inches. Radius of the turn is 1/|kappa|.
     */
    default double curvatureAt(double t) {
        Vec2 d1 = derivativeAt(t);
        Vec2 d2 = secondDerivativeAt(t);
        double speedSq = d1.x * d1.x + d1.y * d1.y;
        double denom = Math.pow(speedSq, 1.5);
        if (denom < 1e-9) return 0;
        return (d1.x * d2.y - d1.y * d2.x) / denom;
    }

    /** Unit tangent at t. Falls back to +x if the derivative vanishes (cusp). */
    default Vec2 tangentAt(double t) {
        Vec2 d = derivativeAt(t);
        double m = Math.hypot(d.x, d.y);
        if (m < 1e-9) return new Vec2(1, 0);
        return new Vec2(d.x / m, d.y / m);
    }
}

/** Immutable 2D point/vector. Small enough to stay readable inline. */
final class Vec2 {
    final double x, y;

    Vec2(double x, double y) { this.x = x; this.y = y; }

    Vec2 plus(Vec2 o)          { return new Vec2(x + o.x, y + o.y); }
    Vec2 minus(Vec2 o)         { return new Vec2(x - o.x, y - o.y); }
    Vec2 times(double s)       { return new Vec2(x * s, y * s); }
    double dot(Vec2 o)         { return x * o.x + y * o.y; }
    double norm()              { return Math.hypot(x, y); }
    double distanceTo(Vec2 o)  { return Math.hypot(x - o.x, y - o.y); }

    @Override public String toString() { return String.format("(%.2f, %.2f)", x, y); }
}

/**
 * Straight line segment. Included as a first-class path type rather than a
 * degenerate bezier because it is exact, cheap, and by far the most common
 * segment in a real auto routine.
 */
class LinePath implements HolonomicPath {
    private Vec2 start, end;

    LinePath(Vec2 start, Vec2 end) { this.start = start; this.end = end; }

    @Override public Vec2 pointAt(double t) {
        t = clamp01(t);
        return start.plus(end.minus(start).times(t));
    }
    /** Constant: the line has no curvature and uniform parameter speed. */
    @Override public Vec2 derivativeAt(double t)       { return end.minus(start); }
    @Override public Vec2 secondDerivativeAt(double t) { return new Vec2(0, 0); }
    @Override public String name()                     { return "Line"; }
    @Override public Vec2[] controlPoints()            { return new Vec2[]{start, end}; }

    @Override public void setControlPoint(int i, Vec2 p) {
        if (i == 0) start = p; else if (i == 1) end = p;
    }

    static double clamp01(double t) { return t < 0 ? 0 : (t > 1 ? 1 : t); }
}

/**
 * Cubic bezier, the Pedro Pathing style curve.
 *   B(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
 *
 * The curve passes through P0 and P3 only; P1 and P2 pull it without being
 * touched. That is exactly why it is nice to author by hand and why a
 * visualizer helps so much here.
 */
class CubicBezierPath implements HolonomicPath {
    private final Vec2[] p = new Vec2[4];

    CubicBezierPath(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3) {
        p[0] = p0; p[1] = p1; p[2] = p2; p[3] = p3;
    }

    @Override public Vec2 pointAt(double t) {
        t = LinePath.clamp01(t);
        double u = 1 - t;
        double b0 = u * u * u;
        double b1 = 3 * u * u * t;
        double b2 = 3 * u * t * t;
        double b3 = t * t * t;
        return new Vec2(
                b0 * p[0].x + b1 * p[1].x + b2 * p[2].x + b3 * p[3].x,
                b0 * p[0].y + b1 * p[1].y + b2 * p[2].y + b3 * p[3].y);
    }

    /** B'(t) = 3(1-t)^2 (P1-P0) + 6(1-t)t (P2-P1) + 3t^2 (P3-P2) */
    @Override public Vec2 derivativeAt(double t) {
        t = LinePath.clamp01(t);
        double u = 1 - t;
        Vec2 a = p[1].minus(p[0]).times(3 * u * u);
        Vec2 b = p[2].minus(p[1]).times(6 * u * t);
        Vec2 c = p[3].minus(p[2]).times(3 * t * t);
        return a.plus(b).plus(c);
    }

    /** B''(t) = 6(1-t)(P2 - 2P1 + P0) + 6t(P3 - 2P2 + P1) */
    @Override public Vec2 secondDerivativeAt(double t) {
        t = LinePath.clamp01(t);
        double u = 1 - t;
        Vec2 a = p[0].plus(p[2]).minus(p[1].times(2)).times(6 * u);
        Vec2 b = p[1].plus(p[3]).minus(p[2].times(2)).times(6 * t);
        return a.plus(b);
    }

    @Override public String name()          { return "Cubic Bezier"; }
    @Override public Vec2[] controlPoints() { return new Vec2[]{p[0], p[1], p[2], p[3]}; }
    @Override public void setControlPoint(int i, Vec2 v) { if (i >= 0 && i < 4) p[i] = v; }
}

/**
 * Quintic Hermite spline segment. Defined by endpoint POSITION, VELOCITY, and
 * ACCELERATION rather than by pulled control points.
 *
 * Why quintic and not cubic: matching acceleration at the endpoints makes
 * curvature continuous across joined segments (C2). A cubic Hermite chain is
 * only C1, so curvature jumps at each joint, and the drivetrain feels that jump
 * as a sudden demand for lateral force. On a mecanum robot that shows up as a
 * skid at the seam between segments.
 *
 * Coefficients derived directly from the six boundary conditions:
 *   c0=p0, c1=v0, c2=a0/2
 *   with A = p1-p0-v0-a0/2,  B = v1-v0-a0,  C = a1-a0
 *   c3 = 10A - 4B + C/2
 *   c4 = -15A + 7B - C
 *   c5 = 6A - 3B + C/2
 * Sanity check: p0=0,p1=1, zero v and a gives 10t^3-15t^4+6t^5, the classic
 * smoothstep, which is verified in the test harness.
 */
class QuinticHermitePath implements HolonomicPath {
    private Vec2 p0, v0, a0, p1, v1, a1;
    private double[] cx = new double[6];
    private double[] cy = new double[6];

    QuinticHermitePath(Vec2 p0, Vec2 v0, Vec2 a0, Vec2 p1, Vec2 v1, Vec2 a1) {
        this.p0 = p0; this.v0 = v0; this.a0 = a0;
        this.p1 = p1; this.v1 = v1; this.a1 = a1;
        recompute();
    }

    /** Convenience: endpoints plus tangent directions, zero endpoint acceleration. */
    static QuinticHermitePath fromTangents(Vec2 start, Vec2 startTangent,
                                           Vec2 end, Vec2 endTangent) {
        return new QuinticHermitePath(start, startTangent, new Vec2(0, 0),
                end, endTangent, new Vec2(0, 0));
    }

    private void recompute() {
        cx = solveAxis(p0.x, v0.x, a0.x, p1.x, v1.x, a1.x);
        cy = solveAxis(p0.y, v0.y, a0.y, p1.y, v1.y, a1.y);
    }

    private static double[] solveAxis(double p0, double v0, double a0,
                                      double p1, double v1, double a1) {
        double c0 = p0, c1 = v0, c2 = a0 / 2.0;
        double A = p1 - p0 - v0 - a0 / 2.0;
        double B = v1 - v0 - a0;
        double C = a1 - a0;
        double c3 = 10 * A - 4 * B + C / 2.0;
        double c4 = -15 * A + 7 * B - C;
        double c5 = 6 * A - 3 * B + C / 2.0;
        return new double[]{c0, c1, c2, c3, c4, c5};
    }

    @Override public Vec2 pointAt(double t) {
        t = LinePath.clamp01(t);
        return new Vec2(poly(cx, t), poly(cy, t));
    }
    @Override public Vec2 derivativeAt(double t) {
        t = LinePath.clamp01(t);
        return new Vec2(dPoly(cx, t), dPoly(cy, t));
    }
    @Override public Vec2 secondDerivativeAt(double t) {
        t = LinePath.clamp01(t);
        return new Vec2(ddPoly(cx, t), ddPoly(cy, t));
    }

    private static double poly(double[] c, double t) {
        return c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))));
    }
    private static double dPoly(double[] c, double t) {
        return c[1] + t * (2 * c[2] + t * (3 * c[3] + t * (4 * c[4] + t * 5 * c[5])));
    }
    private static double ddPoly(double[] c, double t) {
        return 2 * c[2] + t * (6 * c[3] + t * (12 * c[4] + t * 20 * c[5]));
    }

    @Override public String name() { return "Quintic Hermite"; }

    /** Scales a tangent vector to a draggable handle offset, and back. */
    private static final double HANDLE_SCALE = 0.33;

    /**
     * Ordered {p0, handle0, handle1, p1} to match the cubic bezier convention:
     * index 0 and index 3 are the real endpoints, 1 and 2 are pull handles.
     * Keeping the two path types consistent means the visualizer can colour
     * endpoints and draw the handle hull with one code path instead of
     * special-casing, and it stops handles being drawn as if they were
     * endpoints the robot passes through.
     */
    @Override public Vec2[] controlPoints() {
        return new Vec2[]{
                p0,
                p0.plus(v0.times(HANDLE_SCALE)),
                p1.minus(v1.times(HANDLE_SCALE)),
                p1
        };
    }

    @Override public void setControlPoint(int i, Vec2 v) {
        switch (i) {
            case 0: p0 = v; break;
            case 1: v0 = v.minus(p0).times(1 / HANDLE_SCALE); break;
            case 2: v1 = p1.minus(v).times(1 / HANDLE_SCALE); break;
            case 3: p1 = v; break;
            default: return;
        }
        recompute();
    }
}

/**
 * Converts between path parameter t and arc length s.
 *
 * This exists because t is not distance. Stepping t uniformly along a bezier
 * produces a robot that speeds up and slows down for no reason. The table
 * samples the curve densely, accumulates chord lengths, and inverts by binary
 * search so you can ask "what t is 30 inches along?".
 *
 * Chord accumulation slightly UNDER-estimates true arc length (a chord is
 * shorter than the arc it subtends). With the default sample count the error is
 * far below odometry noise, but it is an approximation, not an exact integral.
 */
class ArcLengthTable {
    private static final int DEFAULT_SAMPLES = 400;

    private final double[] tSamples;
    private final double[] sSamples;
    private final double totalLength;

    ArcLengthTable(HolonomicPath path) { this(path, DEFAULT_SAMPLES); }

    ArcLengthTable(HolonomicPath path, int samples) {
        if (samples < 2) throw new IllegalArgumentException("need >= 2 samples");
        tSamples = new double[samples];
        sSamples = new double[samples];

        Vec2 prev = path.pointAt(0);
        tSamples[0] = 0;
        sSamples[0] = 0;
        double acc = 0;
        for (int i = 1; i < samples; i++) {
            double t = (double) i / (samples - 1);
            Vec2 cur = path.pointAt(t);
            acc += cur.distanceTo(prev);
            tSamples[i] = t;
            sSamples[i] = acc;
            prev = cur;
        }
        totalLength = acc;
    }

    double totalLength() { return totalLength; }

    /** Parameter t at arc length s (inches from path start). */
    double tAtArcLength(double s) {
        if (s <= 0) return 0;
        if (s >= totalLength) return 1;

        int lo = 0, hi = sSamples.length - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (sSamples[mid] <= s) lo = mid; else hi = mid;
        }
        double span = sSamples[hi] - sSamples[lo];
        double frac = span < 1e-12 ? 0 : (s - sSamples[lo]) / span;
        return tSamples[lo] + frac * (tSamples[hi] - tSamples[lo]);
    }

    /** Arc length at parameter t. */
    double arcLengthAt(double t) {
        t = LinePath.clamp01(t);
        int lo = 0, hi = tSamples.length - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (tSamples[mid] <= t) lo = mid; else hi = mid;
        }
        double span = tSamples[hi] - tSamples[lo];
        double frac = span < 1e-12 ? 0 : (t - tSamples[lo]) / span;
        return sSamples[lo] + frac * (sSamples[hi] - sSamples[lo]);
    }

    /**
     * Arc length of the closest point on the path to a query position.
     * Used to re-anchor the follower when the robot has been pushed off path.
     * Brute force over the samples: ~400 distance checks, trivial at 50 Hz.
     */
    double closestArcLength(HolonomicPath path, Vec2 query) {
        double best = 0, bestDist = Double.MAX_VALUE;
        for (int i = 0; i < tSamples.length; i++) {
            double d = path.pointAt(tSamples[i]).distanceTo(query);
            if (d < bestDist) { bestDist = d; best = sSamples[i]; }
        }
        return best;
    }
}
/**
 * here is my fav tame impala song for absolutely no reason other than....   (I contemplated if it's then or than for 10 minutes)
 * i am about to lose it!
 * Guess it from this lyric for bragging rights:
 *      Well, he feels like an elephant
 *      Shakin' his big grey trunk for the hell of it
 *      He knows that you're dreamin' about being loved by him
 *      Too bad your chances are slim (DUDU)
 * */