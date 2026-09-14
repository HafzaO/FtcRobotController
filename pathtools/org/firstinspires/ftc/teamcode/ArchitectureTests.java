package org.firstinspires.ftc.teamcode;

import java.util.List;

public class ArchitectureTests {
    static int pass=0, fail=0;
    static void check(String l, boolean ok, String d){
        if(ok){pass++;System.out.println("  PASS  "+l);}
        else{fail++;System.out.println("  FAIL  "+l+"   "+d);} }
    static void near(String l,double g,double w,double t){
        check(l,Math.abs(g-w)<t,String.format("got %.6f want %.6f",g,w)); }

    public static void main(String[] a){
        System.out.println("== PathChain: arc length is additive ==");
        LinePath l1 = new LinePath(new Vec2(-48,-48), new Vec2(0,-48));   // 48
        LinePath l2 = new LinePath(new Vec2(0,-48), new Vec2(0,0));       // 48
        PathChain chain = new PathChain(l1, l2);
        near("chain length = sum of segments", chain.totalLength(), 96.0, 0.05);
        near("chain start = seg0 start", chain.pointAt(0).distanceTo(new Vec2(-48,-48)), 0, 1e-6);
        near("chain end = seg1 end", chain.pointAt(1).distanceTo(new Vec2(0,0)), 0, 1e-6);
        near("chain midpoint at the joint", chain.pointAt(0.5).distanceTo(new Vec2(0,-48)), 0, 0.3);

        System.out.println("\n== PathChain: t distributes by ARC LENGTH not segment count ==");
        // 90in segment then 10in segment. Even-per-segment would put the joint
        // at t=0.5; correct behaviour puts it at t=0.9.
        PathChain lop = new PathChain(
                new LinePath(new Vec2(0,0), new Vec2(90,0)),
                new LinePath(new Vec2(90,0), new Vec2(100,0)));
        near("joint sits at t=0.9", lop.pointAt(0.9).x, 90.0, 0.4);
        check("t=0.5 is mid-FIRST-segment, not the joint",
              Math.abs(lop.pointAt(0.5).x - 50.0) < 0.5,
              "got x="+lop.pointAt(0.5).x);

        System.out.println("\n== PathChain: continuity validation ==");
        PathChain smooth = new PathChain(
                new LinePath(new Vec2(0,0), new Vec2(40,0)),
                new LinePath(new Vec2(40,0), new Vec2(80,0)));
        check("collinear chain reports smooth", smooth.isSmooth(), "");
        PathChain kinked = new PathChain(
                new LinePath(new Vec2(0,0), new Vec2(40,0)),
                new LinePath(new Vec2(40,0), new Vec2(40,40)));   // 90 deg corner
        check("90 degree corner reported NOT smooth", !kinked.isSmooth(), "");
        List<PathChain.Joint> js = kinked.validate();
        near("tangent break measured as 90 deg", Math.toDegrees(js.get(0).tangentBreakRadians), 90, 1.0);
        PathChain gapped = new PathChain(
                new LinePath(new Vec2(0,0), new Vec2(40,0)),
                new LinePath(new Vec2(46,0), new Vec2(80,0)));    // 6in gap
        near("position gap detected", gapped.validate().get(0).positionGapInches, 6.0, 0.05);
        check("gapped chain reported NOT smooth", !gapped.isSmooth(), "");

        System.out.println("\n== PathChain nests, and works as a plain path ==");
        PathChain nested = new PathChain(chain, new LinePath(new Vec2(0,0), new Vec2(0,24)));
        near("nested chain length", nested.totalLength(), 120.0, 0.1);
        LQRPathFollower f0 = LQRPathFollower.withDefaults(40,40,3);
        f0.followPath(nested, 0, 0);   // no API change needed to follow a chain
        near("follower measures the chain", f0.pathLength(), 120.0, 0.2);
        check("curvature delegates to segment (line = 0)", chain.curvatureAt(0.25)==0.0, "");

        System.out.println("\n== ChassisDynamics: voltage feedforward ==");
        ChassisDynamics cd = new ChassisDynamics(0.08, 0.0155, 0.0022, 14, 14, 12.0, 12.0, 14, 0.45);
        near("max achievable velocity = (V-Ks)/Kv", cd.maxAchievableVelocity(), (12-0.08)/0.0155, 1e-6);
        double[] wv = cd.wheelVelocities(20, 0, 0);
        check("pure forward drives all wheels equally",
              Math.abs(wv[0]-20)<1e-9 && Math.abs(wv[1]-20)<1e-9
              && Math.abs(wv[2]-20)<1e-9 && Math.abs(wv[3]-20)<1e-9, "");
        double[] wr = cd.wheelVelocities(0, 0, 1.0);
        check("pure rotation opposes left and right",
              wr[0]>0 && wr[1]<0 && wr[2]>0 && wr[3]<0,
              String.format("%.2f %.2f %.2f %.2f", wr[0],wr[1],wr[2],wr[3]));
        double[] ws = cd.wheelVelocities(0, 20, 0);
        check("pure strafe opposes diagonal pairs",
              ws[0]>0 && ws[1]<0 && ws[2]<0 && ws[3]>0, "");

        System.out.println("\n== ChassisDynamics: voltage compensation is capped ==");
        double[] nom = cd.toMotorPowers(20,0,0, 0,0,0, 12.0);
        double[] sag = cd.toMotorPowers(20,0,0, 0,0,0, 9.0);
        check("sagging battery raises commanded power", sag[0] > nom[0],
              String.format("nom %.4f sag %.4f", nom[0], sag[0]));
        double[] collapse = cd.toMotorPowers(20,0,0, 0,0,0, 4.0);
        double ratio = collapse[0] / nom[0];
        check("boost capped at 1.25x during collapse", ratio <= 1.2501,
              String.format("ratio %.4f", ratio));
        double[] badRead = cd.toMotorPowers(20,0,0, 0,0,0, 0.0);
        check("zero voltage reading fails safe to nominal",
              Math.abs(badRead[0]-nom[0])<1e-9, "");
        double[] still = cd.toMotorPowers(0,0,0, 0,0,0, 12.0);
        check("Ks injects no kick when fully stopped",
              still[0]==0 && still[1]==0 && still[2]==0 && still[3]==0, "");

        System.out.println("\n== ChassisDynamics rejects unmeasured constants ==");
        try{ new ChassisDynamics(0.08,0,0.002,14,14,12,12,14,0.45); check("rejects kv=0",false,"no throw"); }
        catch(IllegalArgumentException e){ check("rejects kv=0",true,""); }
        try{ new ChassisDynamics(0.08,0.015,0.002,0,14,12,12,14,0.45); check("rejects zero track width",false,"no throw"); }
        catch(IllegalArgumentException e){ check("rejects zero track width",true,""); }

        System.out.println("\n== Latency preview: bounded, and does NOT cut corners ==");
        LQRPathFollower f = LQRPathFollower.withDefaults(40,40,3);
        try{ f.setLatencyCompensation(0.4); check("rejects pure-pursuit-sized preview",false,"no throw"); }
        catch(IllegalArgumentException e){ check("rejects pure-pursuit-sized preview",true,""); }
        try{ f.setLatencyCompensation(-0.01); check("rejects negative preview",false,"no throw"); }
        catch(IllegalArgumentException e){ check("rejects negative preview",true,""); }
        f.setLatencyCompensation(0.04);
        CubicBezierPath curve = new CubicBezierPath(new Vec2(-40,-40), new Vec2(-10,40),
                                                    new Vec2(20,-40), new Vec2(40,24));
        f.followPath(curve, 0, 0);
        // The previewed reference must still lie ON the path, which is the
        // property distance-based look-ahead gives up.
        ArcLengthTable at = new ArcLengthTable(curve);
        double worstOff = 0;
        for (double t=0; t<f.duration(); t+=0.05){
            LQRPathFollower.Reference r = f.referenceAt(t + 0.04);
            double best = Double.MAX_VALUE;
            // 20k samples: at 400 the measurement is dominated by grid spacing
            // (0.29in) and reports a deviation that is purely an artefact of
            // the test, not of the controller.
            for(int i=0;i<=20000;i++) best = Math.min(best, curve.pointAt(i/20000.0).distanceTo(r.position));
            worstOff = Math.max(worstOff, best);
        }
        check("previewed reference stays on the path (<0.01in)", worstOff < 0.01,
              String.format("worst %.4f in", worstOff));

        System.out.println("\n== DriverAssist: authority budget ==");
        DriverAssist da = DriverAssist.withDefaults(3.0);
        check("starts unlocked", !da.isLocked(), "");
        da.snapToNearest(Math.toRadians(80), 4);
        near("snaps to nearest 90 deg", Math.toDegrees(da.lockedHeading()), 90, 1e-6);
        da.snapToNearest(Math.toRadians(-100), 4);
        near("snaps negative to -90", Math.toDegrees(da.lockedHeading()), -90, 1e-6);

        // Full stick with a large heading error: rotation must SURVIVE, which
        // is the failure mode the budget exists to prevent.
        da.lockHeading(Math.PI/2);
        double[] p = da.update(1.0, 1.0, 0.0, 0.0);
        double rot = (p[0]-p[1]+p[2]-p[3])/4.0;
        check("rotation survives full-stick translation", Math.abs(rot) > 0.05,
              String.format("rotation component %.4f", rot));
        da.release();
        double[] p2 = da.update(1.0, 1.0, 0.0, 0.0);
        double rot2 = (p2[0]-p2[1]+p2[2]-p2[3])/4.0;
        check("no rotation injected when released", Math.abs(rot2) < 1e-9,
              String.format("%.6f", rot2));
        try{ new DriverAssist(8,1,0.02,3.0,1.5); check("rejects authority >= 1",false,"no throw"); }
        catch(IllegalArgumentException e){ check("rejects authority >= 1",true,""); }

        System.out.println("\n== Score macro geometry ==");
        HolonomicPath macro = DriverAssist.scoreMacroPath(0,0,0, 30,20, Math.PI/2);
        near("macro starts at robot", macro.pointAt(0).distanceTo(new Vec2(0,0)), 0, 1e-9);
        near("macro ends at target", macro.pointAt(1).distanceTo(new Vec2(30,20)), 0, 1e-9);
        Vec2 t0 = macro.tangentAt(0);
        near("leaves along current heading", Math.atan2(t0.y,t0.x), 0, 0.02);
        Vec2 t1 = macro.tangentAt(1);
        near("arrives along target heading", Math.atan2(t1.y,t1.x), Math.PI/2, 0.02);

        System.out.println("\n== Localizer unit contract ==");
        near("mm to inches", Localizer.mmToInches(254.0), 10.0, 1e-12);
        near("inches to mm", Localizer.inchesToMm(10.0), 254.0, 1e-12);
        SimulatedLocalizer sim = new SimulatedLocalizer(5,6,0.5);
        near("sim reports pose x", sim.getPose().x, 5, 1e-12);
        sim.setPose(new Localizer.Pose(1,2,3));
        near("setPose applies", sim.getPose().y, 2, 1e-12);

        System.out.printf("%n===== %d passed, %d failed =====%n", pass, fail);
        if(fail>0) System.exit(1);
    }
}
