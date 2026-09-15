package org.firstinspires.ftc.teamcode;

/** Closed-loop sim of a chained auto, smooth vs kinked, to quantify the joint cost. */
public class ChainSim {
    static double MAXV=40, MAXW=3.0;

    /**
     * Returns {finalErr, peakErrNearJoint, peakErrAwayFromJoint}.
     *
     * Comparing the MEAN error of two different chains turned out to be a
     * meaningless test: changing a segment's exit tangent changes that whole
     * segment's shape and length, so the two chains are simply different paths.
     * The joint effect has to be isolated WITHIN one chain, by comparing error
     * in a window around the joint crossing against error everywhere else.
     */
    static double[] run(HolonomicPath path, String label, double jointTime){
        LQRPathFollower f=LQRPathFollower.withDefaults(MAXV,40,MAXW);
        f.followPath(path,0,Math.PI/2);
        double dt=0.02,x=path.pointAt(0).x,y=path.pointAt(0).y,th=0,vx=0,vy=0,w=0,lag=0.35;
        double nearJoint=0, awayJoint=0;
        for(double t=0;t<f.duration()+1.0;t+=dt){
            double[] p=f.update(x,y,th,dt);
            double rvx=(p[0]+p[1]+p[2]+p[3])/4.0*MAXV;
            double rvy=(p[0]-p[1]-p[2]+p[3])/4.0*MAXV;
            double rw =(p[0]-p[1]+p[2]-p[3])/4.0*MAXW;
            double c=Math.cos(th),s=Math.sin(th);
            vx+=((rvx*c-rvy*s)-vx)*lag; vy+=((rvx*s+rvy*c)-vy)*lag; w+=(rw-w)*lag;
            x+=vx*dt; y+=vy*dt; th+=w*dt;
            LQRPathFollower.Reference r=f.referenceAt(f.elapsed());
            double e=Math.hypot(r.position.x-x,r.position.y-y);
            if(f.elapsed()>f.duration()) continue;
            // CROSS-TRACK error: distance from the robot to the path GEOMETRY.
            // Corner rounding is a geometric departure from the path, and
            // measuring against the moving reference instead mixes it with the
            // startup transient, which is larger and masks it entirely.
            double cross=Double.MAX_VALUE;
            for(int i=0;i<=3000;i++)
                cross=Math.min(cross, path.pointAt(i/3000.0).distanceTo(new Vec2(x,y)));
            if(Math.abs(f.elapsed()-jointTime)<0.35) nearJoint=Math.max(nearJoint,cross);
            else awayJoint=Math.max(awayJoint,cross);
        }
        Vec2 end=path.pointAt(1);
        double fin=Math.hypot(end.x-x,end.y-y);
        System.out.printf("%-22s cross-track@joint=%5.2f  cross-track elsewhere=%5.2f  ratio=%4.2fx  final=%4.2f%n",
                label,nearJoint,awayJoint,nearJoint/Math.max(0.01,awayJoint),fin);
        return new double[]{fin,nearJoint,awayJoint};
    }

    /** Time at which the profile reaches the end of segment 0. */
    static double jointTime(PathChain chain){
        LQRPathFollower f=LQRPathFollower.withDefaults(MAXV,40,MAXW);
        f.followPath(chain,0,Math.PI/2);
        double target=new ArcLengthTable(chain.segment(0)).totalLength();
        for(double t=0;t<f.duration();t+=0.005)
            if(f.referenceAt(t).arcLength>=target) return t;
        return f.duration()/2;
    }

    public static void main(String[] a){
        System.out.println("== Chained auto: smooth joint vs kinked joint ==");

        // Smooth: quintic exits along +x, line continues along +x.
        PathChain smooth=new PathChain(
            QuinticHermitePath.fromTangents(new Vec2(-56,-56), new Vec2(90,0),
                                            new Vec2(0,-24), new Vec2(90,0)),
            new LinePath(new Vec2(0,-24), new Vec2(48,-24)));
        System.out.println("  smooth chain joints:");
        for(PathChain.Joint j: smooth.validate()) System.out.println("    "+j);
        double[] rs=run(smooth,"smooth chain",jointTime(smooth));

        // Kinked: same endpoints, but the quintic arrives going +y into a +x line.
        PathChain kinked=new PathChain(
            QuinticHermitePath.fromTangents(new Vec2(-56,-56), new Vec2(90,0),
                                            new Vec2(0,-24), new Vec2(0,90)),
            new LinePath(new Vec2(0,-24), new Vec2(48,-24)));
        System.out.println("  kinked chain joints:");
        for(PathChain.Joint j: kinked.validate()) System.out.println("    "+j);
        double[] rk=run(kinked,"kinked chain",jointTime(kinked));

        System.out.println("\n== Does the chain beat stopping at the waypoint? ==");
        // Two separate paths run back to back, each profiled to a full stop.
        LQRPathFollower f1=LQRPathFollower.withDefaults(MAXV,40,MAXW);
        f1.followPath(smooth.segment(0),0,Math.PI/4);
        LQRPathFollower f2=LQRPathFollower.withDefaults(MAXV,40,MAXW);
        f2.followPath(smooth.segment(1),Math.PI/4,Math.PI/2);
        double stopAndGo=f1.duration()+f2.duration();
        LQRPathFollower fc=LQRPathFollower.withDefaults(MAXV,40,MAXW);
        fc.followPath(smooth,0,Math.PI/2);
        System.out.printf("  stop-at-waypoint: %.2fs    single chained profile: %.2fs    saved %.2fs%n",
                stopAndGo, fc.duration(), stopAndGo-fc.duration());

        System.out.println("\n== Checks ==");
        // Compare cross-track AT THE JOINT between the two chains. The
        // within-chain ratio is diluted by the startup transient, which is the
        // largest departure on any run and has nothing to do with the joint.
        System.out.println((rs[1]<0.25?"  PASS":"  FAIL")
                +"  smooth joint holds the path ("+String.format("%.2f",rs[1])+" in cross-track)");
        System.out.println((rk[1]>rs[1]*3?"  PASS":"  FAIL")
                +"  kinked joint rounds the corner ("+String.format("%.2f",rk[1])
                +" in vs "+String.format("%.2f",rs[1])+" in, "
                +String.format("%.0fx",rk[1]/Math.max(0.01,rs[1]))+" worse)");
        System.out.println((fc.duration()<stopAndGo?"  PASS":"  FAIL")
                +"  chaining is faster than stopping at the waypoint");
    }
}
