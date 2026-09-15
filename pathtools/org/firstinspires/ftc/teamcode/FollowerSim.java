package org.firstinspires.ftc.teamcode;

/** Closed-loop simulation. Plant: first-order velocity lag, matching a real DT. */
public class FollowerSim {
    static double MAXV = 40, MAXW = 3.0;

    static double[] run(HolonomicPath path, double h0, double h1,
                        double startX, double startY, double startH,
                        String label, boolean verbose) {
        LQRPathFollower f = LQRPathFollower.withDefaults(MAXV, 40, MAXW);
        f.followPath(path, h0, h1);

        double dt = 0.02, x = startX, y = startY, th = startH;
        double vx = 0, vy = 0, w = 0;
        double lag = 0.35; // first-order response toward commanded velocity
        double maxErr = 0, sumErr = 0; int n = 0;

        double T = f.duration() + 1.0;
        for (double t = 0; t < T; t += dt) {
            double[] p = f.update(x, y, th, dt);

            // invert mecanum mixer -> robot-frame normalized velocities
            double rvx = (p[0] + p[1] + p[2] + p[3]) / 4.0 * MAXV;
            double rvy = (p[0] - p[1] - p[2] + p[3]) / 4.0 * MAXV;
            double rw  = (p[0] - p[1] + p[2] - p[3]) / 4.0 * MAXW;

            // robot frame -> field frame
            double c = Math.cos(th), s = Math.sin(th);
            double cmdVx = rvx * c - rvy * s;
            double cmdVy = rvx * s + rvy * c;

            vx += (cmdVx - vx) * lag;
            vy += (cmdVy - vy) * lag;
            w  += (rw   - w ) * lag;
            x += vx * dt; y += vy * dt; th += w * dt;

            LQRPathFollower.Reference ref = f.referenceAt(f.elapsed());
            double e = Math.hypot(ref.position.x - x, ref.position.y - y);
            if (f.elapsed() <= f.duration()) { maxErr = Math.max(maxErr, e); sumErr += e; n++; }
        }
        Vec2 end = path.pointAt(1.0);
        double finalErr = Math.hypot(end.x - x, end.y - y);
        double headErr = Math.abs(LQRPathFollower.normalizeAngle(h1 - th));
        if (verbose) {
            System.out.printf("%-18s len=%6.1f in  dur=%4.2fs  meanErr=%5.2f  maxErr=%5.2f  finalErr=%5.2f in  headErr=%.3f rad%n",
                    label, f.pathLength(), f.duration(), sumErr/Math.max(1,n), maxErr, finalErr, headErr);
        }
        return new double[]{finalErr, maxErr, headErr};
    }

    public static void main(String[] a) {
        System.out.println("== Tracking from ON the path start ==");
        run(new LinePath(new Vec2(-48,-48), new Vec2(48,24)), 0, Math.PI/2,
            -48,-48,0, "Line", true);
        run(new CubicBezierPath(new Vec2(-48,-48), new Vec2(-10,40),
                                new Vec2(20,-40), new Vec2(48,24)), 0, Math.PI,
            -48,-48,0, "Cubic Bezier", true);
        run(QuinticHermitePath.fromTangents(new Vec2(-48,-48), new Vec2(120,0),
                                            new Vec2(48,24), new Vec2(0,120)),
            0, -Math.PI/2, -48,-48,0, "Quintic Hermite", true);

        System.out.println("\n== Disturbance rejection: start 15in OFF the path ==");
        double[] r = run(new CubicBezierPath(new Vec2(-48,-48), new Vec2(-10,40),
                                new Vec2(20,-40), new Vec2(48,24)), 0, Math.PI,
            -48+15,-48-8,0.3, "Bezier (pushed)", true);

        System.out.println("\n== Checks ==");
        boolean ok1 = r[0] < 3.0;
        System.out.println((ok1?"  PASS":"  FAIL") + "  recovers from 17in initial offset (final err "
                           + String.format("%.2f", r[0]) + " in)");
        double[] r2 = run(new LinePath(new Vec2(-48,-48), new Vec2(48,24)), 0, Math.PI/2,
            -48,-48,0, "", false);
        boolean ok2 = r2[0] < 2.0 && r2[1] < 6.0;
        System.out.println((ok2?"  PASS":"  FAIL") + "  line tracking stays bounded (max "
                           + String.format("%.2f", r2[1]) + " in, final " + String.format("%.2f", r2[0]) + " in)");
        boolean ok3 = r2[2] < 0.1;
        System.out.println((ok3?"  PASS":"  FAIL") + "  heading converges (err "
                           + String.format("%.4f", r2[2]) + " rad)");

        System.out.println("\n== Short-path (triangular profile) regression ==");
        double[] r3 = run(new LinePath(new Vec2(0,0), new Vec2(6,0)), 0, 0, 0,0,0, "6in hop", true);
        System.out.println((r3[0] < 1.5 ?"  PASS":"  FAIL") + "  short move converges");
    }
}
