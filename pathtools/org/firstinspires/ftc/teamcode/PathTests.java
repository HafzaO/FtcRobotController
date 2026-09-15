package org.firstinspires.ftc.teamcode;

public class PathTests {
    static int pass = 0, fail = 0;

    static void check(String label, boolean ok, String detail) {
        if (ok) { pass++; System.out.println("  PASS  " + label); }
        else    { fail++; System.out.println("  FAIL  " + label + "   " + detail); }
    }
    static void near(String label, double got, double want, double tol) {
        check(label, Math.abs(got - want) < tol,
              String.format("got %.6f want %.6f", got, want));
    }

    /** Exact closed-form solution of the scalar DARE, for cross-checking. */
    static double exactGain(double b, double q, double r) {
        double p = (q*b*b + Math.sqrt(q*q*Math.pow(b,4) + 4*b*b*q*r)) / (2*b*b);
        return b*p / (r + p*b*b);
    }

    public static void main(String[] a) {
        System.out.println("== Quintic Hermite: smoothstep identity ==");
        // p0=0,p1=1, zero velocity and acceleration -> 10t^3 - 15t^4 + 6t^5
        QuinticHermitePath q = new QuinticHermitePath(
                new Vec2(0,0), new Vec2(0,0), new Vec2(0,0),
                new Vec2(1,0), new Vec2(0,0), new Vec2(0,0));
        for (double t : new double[]{0, 0.25, 0.5, 0.75, 1.0}) {
            double want = 10*Math.pow(t,3) - 15*Math.pow(t,4) + 6*Math.pow(t,5);
            near("smoothstep t="+t, q.pointAt(t).x, want, 1e-12);
        }
        near("quintic endpoint velocity is zero", q.derivativeAt(1.0).x, 0, 1e-12);
        near("quintic endpoint accel is zero", q.secondDerivativeAt(1.0).x, 0, 1e-9);

        System.out.println("\n== Quintic honours boundary conditions ==");
        QuinticHermitePath q2 = new QuinticHermitePath(
                new Vec2(3,-4), new Vec2(10,2), new Vec2(1,-1),
                new Vec2(20,15), new Vec2(-3,7), new Vec2(2,2));
        near("p(0).x", q2.pointAt(0).x, 3, 1e-12);
        near("p(0).y", q2.pointAt(0).y, -4, 1e-12);
        near("p(1).x", q2.pointAt(1).x, 20, 1e-10);
        near("p(1).y", q2.pointAt(1).y, 15, 1e-10);
        near("v(0).x", q2.derivativeAt(0).x, 10, 1e-10);
        near("v(1).y", q2.derivativeAt(1).y, 7, 1e-9);
        near("a(0).x", q2.secondDerivativeAt(0).x, 1, 1e-9);
        near("a(1).y", q2.secondDerivativeAt(1).y, 2, 1e-8);

        System.out.println("\n== Control point ordering is consistent ==");
        QuinticHermitePath qc = QuinticHermitePath.fromTangents(
                new Vec2(-48,-48), new Vec2(120,0), new Vec2(48,24), new Vec2(0,120));
        Vec2[] qcp = qc.controlPoints();
        near("quintic cp[0] is start point", qcp[0].distanceTo(qc.pointAt(0)), 0, 1e-9);
        near("quintic cp[3] is end point", qcp[3].distanceTo(qc.pointAt(1)), 0, 1e-9);
        CubicBezierPath bc = new CubicBezierPath(new Vec2(-48,-48), new Vec2(-10,40),
                                new Vec2(20,-40), new Vec2(48,24));
        Vec2[] bcp = bc.controlPoints();
        near("bezier cp[0] is start point", bcp[0].distanceTo(bc.pointAt(0)), 0, 1e-9);
        near("bezier cp[3] is end point", bcp[3].distanceTo(bc.pointAt(1)), 0, 1e-9);
        check("both types put endpoints at index 0 and last",
              qcp.length == 4 && bcp.length == 4, "");
        // dragging handle 1 must not move the endpoints
        qc.setControlPoint(1, new Vec2(0, 0));
        near("dragging handle leaves start fixed", qc.pointAt(0).distanceTo(new Vec2(-48,-48)), 0, 1e-9);
        near("dragging handle leaves end fixed", qc.pointAt(1).distanceTo(new Vec2(48,24)), 0, 1e-8);

        System.out.println("\n== Cubic Bezier endpoints and derivative ==");
        CubicBezierPath b = new CubicBezierPath(
                new Vec2(0,0), new Vec2(10,20), new Vec2(30,20), new Vec2(40,0));
        near("B(0).x", b.pointAt(0).x, 0, 1e-12);
        near("B(1).x", b.pointAt(1).x, 40, 1e-12);
        // B'(0) = 3(P1-P0)
        near("B'(0).x", b.derivativeAt(0).x, 30, 1e-10);
        near("B'(0).y", b.derivativeAt(0).y, 60, 1e-10);
        // B'(1) = 3(P3-P2)
        near("B'(1).x", b.derivativeAt(1).x, 30, 1e-10);
        near("B'(1).y", b.derivativeAt(1).y, -60, 1e-10);
        // numeric derivative cross-check
        double h = 1e-6;
        double numDx = (b.pointAt(0.4+h).x - b.pointAt(0.4-h).x) / (2*h);
        near("B'(0.4).x vs numeric", b.derivativeAt(0.4).x, numDx, 1e-4);
        double numDDx = (b.derivativeAt(0.4+h).x - b.derivativeAt(0.4-h).x) / (2*h);
        near("B''(0.4).x vs numeric", b.secondDerivativeAt(0.4).x, numDDx, 1e-4);

        System.out.println("\n== Curvature: circle of known radius ==");
        // Quarter circle radius 20 approximated by bezier with kappa handle 0.5523
        double R = 20, k = 0.5522847498;
        CubicBezierPath circ = new CubicBezierPath(
                new Vec2(R,0), new Vec2(R, R*k), new Vec2(R*k, R), new Vec2(0,R));
        double curv = circ.curvatureAt(0.5);
        near("curvature ~ 1/R at midpoint", Math.abs(curv), 1.0/R, 2e-3);
        check("line curvature is zero",
              new LinePath(new Vec2(0,0), new Vec2(50,50)).curvatureAt(0.5) == 0.0, "");

        System.out.println("\n== Arc length ==");
        LinePath line = new LinePath(new Vec2(0,0), new Vec2(30,40)); // 3-4-5 -> 50
        ArcLengthTable lt = new ArcLengthTable(line);
        near("line length = 50", lt.totalLength(), 50.0, 1e-9);
        near("t at s=25 is 0.5", lt.tAtArcLength(25), 0.5, 1e-6);
        // quarter circle arc length = pi*R/2 = 31.4159
        ArcLengthTable ct = new ArcLengthTable(circ, 2000);
        near("quarter-circle arc length", ct.totalLength(), Math.PI*R/2, 0.02);
        // arc-length reparam should give near-uniform spacing
        double maxGapErr = 0;
        double L = ct.totalLength();
        Vec2 prev = circ.pointAt(ct.tAtArcLength(0));
        for (int i=1;i<=50;i++){
            Vec2 cur = circ.pointAt(ct.tAtArcLength(L*i/50.0));
            maxGapErr = Math.max(maxGapErr, Math.abs(cur.distanceTo(prev) - L/50.0));
            prev = cur;
        }
        check("reparam gives uniform spacing (max gap err < 0.01in)", maxGapErr < 0.01,
              String.format("maxGapErr=%.5f", maxGapErr));
        // and show that RAW t does NOT
        double maxRawErr = 0;
        prev = circ.pointAt(0);
        for (int i=1;i<=50;i++){
            Vec2 cur = circ.pointAt(i/50.0);
            maxRawErr = Math.max(maxRawErr, Math.abs(cur.distanceTo(prev) - L/50.0));
            prev = cur;
        }
        System.out.printf("     (raw-t spacing error %.5f in vs reparam %.5f in)%n", maxRawErr, maxGapErr);

        System.out.println("\n== Trapezoidal profile ==");
        TrapezoidalProfile tp = new TrapezoidalProfile(100, 40, 40); // long: trapezoid
        near("trapezoid ends at length", tp.positionAt(tp.duration()), 100, 1e-6);
        near("trapezoid peak = maxVel", tp.peakVelocity(), 40, 1e-9);
        near("v=0 at start", tp.velocityAt(0), 0, 1e-9);
        near("v=0 at end", tp.velocityAt(tp.duration()), 0, 1e-9);
        TrapezoidalProfile tri = new TrapezoidalProfile(5, 40, 40); // short: triangle
        near("triangle ends at length", tri.positionAt(tri.duration()), 5, 1e-6);
        check("triangle peak < maxVel", tri.peakVelocity() < 40,
              "peak="+tri.peakVelocity());
        near("triangle peak = sqrt(a*L)", tri.peakVelocity(), Math.sqrt(40*5), 1e-9);
        // monotonic position
        boolean mono = true;
        for (double t=0;t<tp.duration();t+=0.01)
            if (tp.positionAt(t+0.01) < tp.positionAt(t) - 1e-9) mono = false;
        check("position monotonically increases", mono, "");
        near("timeAtPosition inverts positionAt", tp.positionAt(tp.timeAtPosition(37.5)), 37.5, 1e-4);

        System.out.println("\n== LQR gains ==");
        LQRPathFollower f = new LQRPathFollower(40, 40, 3, 10, 1, 8, 1, 0.02, 0.7);
        double[][] K = f.gains();
        System.out.printf("     K = [%.4f, %.4f, %.4f]%n", K[0][0], K[1][1], K[2][2]);
        // Compare the ITERATED solver against the exact closed-form solution of
        // the scalar DARE. This is the real correctness test.
        near("xy gain matches closed form", K[0][0], exactGain(0.02, 10, 1), 1e-9);
        near("theta gain matches closed form", K[2][2], exactGain(0.02, 8, 1), 1e-9);
        // sqrt(q/r) is only the dt->0 limit. At dt=0.02 it is ~3% high, so this
        // is asserted loosely and deliberately: it is a tuning heuristic, not
        // an identity.
        near("k near sqrt(q/r) within 5pc", K[0][0], Math.sqrt(10.0), 0.05*Math.sqrt(10.0));
        double kFine = new LQRPathFollower(40,40,3,10,1,8,1,0.0001,0.7).gains()[0][0];
        check("gain converges to sqrt(q/r) as dt shrinks",
              Math.abs(kFine - Math.sqrt(10.0)) < 0.005,
              String.format("dt=1e-4 gain=%.6f", kFine));
        check("K is diagonal (axes decoupled)",
              K[0][1]==0 && K[0][2]==0 && K[1][0]==0 && K[2][1]==0, "");
        // closed loop stability: x[k+1] = (1 - dt*k) x
        double cl = 1 - 0.02*K[0][0];
        check("closed loop stable |1-dt*k|<1", Math.abs(cl) < 1.0, "got "+cl);

        System.out.println("\n== Riccati rejects invalid cost matrices ==");
        try { new LQRPathFollower(40,40,3, 10, 0, 8, 1, 0.02, 0.7); check("rejects r=0", false, "no throw"); }
        catch (IllegalArgumentException e) { check("rejects r=0", true, ""); }
        try { new LQRPathFollower(40,40,3, 0, 1, 8, 1, 0.02, 0.7); check("rejects q=0", false, "no throw"); }
        catch (IllegalArgumentException e) { check("rejects q=0", true, ""); }
        try { new TrapezoidalProfile(10, 0, 40); check("rejects maxVel=0", false, "no throw"); }
        catch (IllegalArgumentException e) { check("rejects maxVel=0", true, ""); }
        try { new LQRPathFollower(40,40,3,10,1,8,1,0.02, 0); check("rejects headroom=0", false, "no throw"); }
        catch (IllegalArgumentException e) { check("rejects headroom=0", true, ""); }
        try { new LQRPathFollower(40,40,3,10,1,8,1,0.02, 1.5); check("rejects headroom>1", false, "no throw"); }
        catch (IllegalArgumentException e) { check("rejects headroom>1", true, ""); }

        System.out.println("\n== Angular feedforward zeroes outside profile ==");
        LQRPathFollower hf = LQRPathFollower.withDefaults(40, 40, 3);
        hf.followPath(new LinePath(new Vec2(0,0), new Vec2(50,0)), 0, Math.PI/2);
        near("omega nonzero mid-path", Math.abs(hf.referenceAt(hf.duration()*0.5).omega) > 1e-6 ? 1 : 0, 1, 1e-9);
        near("omega zero after path ends", hf.referenceAt(hf.duration()*2).omega, 0, 1e-12);
        near("omega zero before start", hf.referenceAt(-1).omega, 0, 1e-12);
        near("heading reaches target at end", hf.referenceAt(hf.duration()).heading, Math.PI/2, 1e-9);

        System.out.printf("%n===== %d passed, %d failed =====%n", pass, fail);
        if (fail > 0) System.exit(1);
    }
}
