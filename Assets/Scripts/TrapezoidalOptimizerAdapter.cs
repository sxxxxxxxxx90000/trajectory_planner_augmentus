using System;
using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{

public class TrapezoidalOptimizerAdapter : AbstractOptimizerAdapter
{
    public override List<TrajectoryPoint> Plan(
        List<double[]> path, double dt, double vMax, double aMax)
    {
        if (path == null || path.Count < 2) throw new ArgumentException("Path must have >= 2 points.");
        if (dt <= 0) throw new ArgumentException("dt must be > 0.");
        if (vMax <= 0 || aMax <= 0) throw new ArgumentException("vMax and aMax must be > 0.");

        int N = path[0].Length;
        for (int i = 1; i < path.Count; i++)
            if (path[i].Length != N) throw new ArgumentException("All waypoints must have same dimension.");

        // Compute arc-lengths
        int segCount = path.Count - 1;
        var segLen = new double[segCount];
        var cum = new double[segCount + 1];
        cum[0] = 0;
        for (int i = 0; i < segCount; i++)
        {
            segLen[i] = Dist(path[i], path[i + 1]);
            cum[i + 1] = cum[i] + segLen[i];
        }
        double L = cum[segCount];
        if (L < 1e-12)
        {
            return new List<TrajectoryPoint> { new TrajectoryPoint(0, Clone(path[0]), Zeros(N)) };
        }

        // Trapezoid profile parameters
        double da = 0.5 * vMax * vMax / aMax;
        bool triangular = (2 * da > L);
        double ta, tc, totalT, vPeak;

        if (triangular)
        {
            vPeak = Math.Sqrt(aMax * L);
            ta = vPeak / aMax;
            tc = 0.0;
            totalT = 2 * ta;
        }
        else
        {
            vPeak = vMax;
            ta = vPeak / aMax;
            double accelDist = 0.5 * aMax * ta * ta;
            double cruiseDist = L - 2 * accelDist;
            tc = cruiseDist / vPeak;
            totalT = 2 * ta + tc;
        }

        // Sampling
        var traj = new List<TrajectoryPoint>();
        int steps = Math.Max(1, (int)Math.Ceiling(totalT / dt));
        for (int k = 0; k <= steps; k++)
        {
            double t = Math.Min(k * dt, totalT);
            double s, sdot;

            if (!triangular)
            {
                if (t <= ta)
                {
                    s = 0.5 * aMax * t * t;
                    sdot = aMax * t;
                }
                else if (t <= ta + tc)
                {
                    double t2 = t - ta;
                    s = 0.5 * aMax * ta * ta + vPeak * t2;
                    sdot = vPeak;
                }
                else
                {
                    double td = t - (ta + tc);
                    s = 0.5 * aMax * ta * ta + vPeak * tc + vPeak * td - 0.5 * aMax * td * td;
                    sdot = Math.Max(0.0, vPeak - aMax * td);
                }
            }
            else
            {
                if (t <= ta)
                {
                    s = 0.5 * aMax * t * t;
                    sdot = aMax * t;
                }
                else
                {
                    double td = t - ta;
                    double sAtPeak = 0.5 * aMax * ta * ta;
                    s = sAtPeak + vPeak * td - 0.5 * aMax * td * td;
                    sdot = Math.Max(0.0, vPeak - aMax * td);
                }
            }

            if (k == steps) { s = L; sdot = 0.0; }

            double[] pos, dirUnit;
            MapArcLengthToPose(path, segLen, cum, s, out pos, out dirUnit);

            double[] vel = Scale(dirUnit, sdot);
            traj.Add(new TrajectoryPoint(t, pos, vel));
        }

        return traj;
    }

    // Helper functions
    private static double Dist(double[] a, double[] b)
    {
        double s = 0;
        for (int i = 0; i < a.Length; i++) { double d = a[i] - b[i]; s += d * d; }
        return Math.Sqrt(s);
    }
    private static double[] Clone(double[] a) { var r = new double[a.Length]; Array.Copy(a, r, a.Length); return r; }
    private static double[] Zeros(int n) => new double[n];
    private static double[] Sub(double[] a, double[] b) { var r = new double[a.Length]; for (int i=0;i<a.Length;i++) r[i]=a[i]-b[i]; return r; }
    private static double[] Scale(double[] v, double s) { var r=new double[v.Length]; for (int i=0;i<v.Length;i++) r[i]=v[i]*s; return r; }
    private static double Norm(double[] v) { double s=0; for(int i=0;i<v.Length;i++) s+=v[i]*v[i]; return Math.Sqrt(s); }
    private static double[] AddScaled(double[] a,double[] dir,double u){var r=new double[a.Length];for(int i=0;i<a.Length;i++)r[i]=a[i]+u*dir[i];return r;}
    private static void MapArcLengthToPose(List<double[]> path,double[] segLen,double[] cum,double s,
        out double[] pos,out double[] dirUnit)
    {
        int segCount = segLen.Length;
        if (s >= cum[segCount])
        {
            pos = Clone(path[segCount]);
            var d = Sub(path[segCount], path[segCount - 1]);
            double len = Norm(d);
            dirUnit = (len > 1e-12) ? Scale(d, 1.0 / len) : Zeros(d.Length);
            return;
        }
        int iSeg = 0;
        for (int i = 0; i < segCount; i++) if (s < cum[i + 1]) { iSeg = i; break; }
        double s0 = cum[iSeg];
        double ds = segLen[iSeg];
        double u = (ds > 1e-12) ? (s - s0) / ds : 0.0;
        var a = path[iSeg];
        var b = path[iSeg + 1];
        var dir = Sub(b, a);
        double lenDir = Norm(dir);
        dirUnit = (lenDir > 1e-12) ? Scale(dir, 1.0 / lenDir) : Zeros(dir.Length);
        pos = AddScaled(a, dirUnit, u * ds);
    }
}


} // namespace Trajectory_Planner_Augmentus