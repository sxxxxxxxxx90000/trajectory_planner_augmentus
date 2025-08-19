using System;
using System.Collections.Generic;
using System.Linq;

namespace Trajectory_Planner_Augmentus
{


public class RRTPlannerAdapter : AbstractPlannerAdapter
{
    public class Node
    {
        public double[] Q;
        public int Parent;   // index into m_nodes; -1 for root
        public Node(double[] q, int parent)
        {
            Q = (double[])q.Clone();
            Parent = parent;
        }
    }

    // Members 
    private readonly Func<double[], bool> m_isFree;
    private readonly Func<double[], double[], bool> m_isSegmentFreeOpt;

    private int m_n = -1;                   // dimension, set in Plan()
    private double[] m_min = null;          // optional bounds (can be set via SetMetaParameters)
    private double[] m_max = null;

    private double m_stepSize = 0.5;
    private double m_goalThreshold = 0.5;
    private int    m_maxIterations = 20000;
    private double m_goalSampleRate = 0.05;
    private double m_segmentResolution = 0.25;
    private bool   m_smooth = true;
    private int    m_smoothIters = 200;

    private Random m_rng = new Random();
    private readonly List<Node> m_nodes = new();

    // Constructor
    public RRTPlannerAdapter(
        Func<double[], bool> isFree,
        Func<double[], double[], bool> isSegmentFreeOpt = null)
    {
        m_isFree = isFree ?? throw new ArgumentNullException(nameof(isFree));
        m_isSegmentFreeOpt = isSegmentFreeOpt;
    }

    // Configure meta parameters for RRT. Pass null to leave a parameter unchanged.
    public void SetMetaParameters(
        double[] minBounds = null,
        double[] maxBounds = null,
        double?  stepSize = null,
        double?  goalThreshold = null,
        int?     maxIterations = null,
        double?  goalSampleRate = null,
        double?  segmentResolution = null,
        int?     randomSeed = null,
        bool?    smooth = null,
        int?     smoothIters = null)
    {
        if (minBounds != null || maxBounds != null)
        {
            if (minBounds == null || maxBounds == null)
                throw new ArgumentException("Both minBounds and maxBounds must be provided together.");
            if (minBounds.Length != maxBounds.Length)
                throw new ArgumentException("Bounds must have the same dimension.");
            for (int i = 0; i < minBounds.Length; i++)
                if (minBounds[i] > maxBounds[i])
                    throw new ArgumentException("minBounds must be <= maxBounds.");

            m_min = (double[])minBounds.Clone();
            m_max = (double[])maxBounds.Clone();
        }

        if (stepSize.HasValue)          m_stepSize = Math.Max(1e-9, stepSize.Value);
        if (goalThreshold.HasValue)     m_goalThreshold = Math.Max(1e-12, goalThreshold.Value);
        if (maxIterations.HasValue)     m_maxIterations = Math.Max(1, maxIterations.Value);
        if (goalSampleRate.HasValue)    m_goalSampleRate = Math.Clamp(goalSampleRate.Value, 0.0, 1.0);
        if (segmentResolution.HasValue) m_segmentResolution = Math.Max(1e-9, segmentResolution.Value);
        if (randomSeed.HasValue)        m_rng = new Random(randomSeed.Value);
        if (smooth.HasValue)            m_smooth = smooth.Value;
        if (smoothIters.HasValue)       m_smoothIters = Math.Max(0, smoothIters.Value);
    }

    // Plan interface
    public override List<double[]> Plan(double[] start, double[] goal)
    {
        if (start == null || goal == null) throw new ArgumentNullException("start/goal cannot be null");
        if (start.Length == 0 || goal.Length == 0) throw new ArgumentException("start/goal must be non-empty");
        if (start.Length != goal.Length) throw new ArgumentException("start/goal dimension mismatch");

        m_n = start.Length;

        // If bounds were not set via SetMetaParameters, derive defaults around start/goal
        if (m_min == null || m_max == null)
        {
            m_min = new double[m_n];
            m_max = new double[m_n];
            for (int i = 0; i < m_n; i++)
            {
                double lo = Math.Min(start[i], goal[i]);
                double hi = Math.Max(start[i], goal[i]);
                double span = Math.Max(1.0, hi - lo);
                m_min[i] = lo - 0.25 * span - 1e-6;
                m_max[i] = hi + 0.25 * span + 1e-6;
            }
        }
        else
        {
            if (m_min.Length != m_n || m_max.Length != m_n)
                throw new ArgumentException("Pre-set bounds dimension does not match start/goal dimension.");
        }

        if (!InBounds(start) || !m_isFree(start))
            throw new ArgumentException("Start is invalid or in collision.");
        if (!InBounds(goal) || !m_isFree(goal))
            throw new ArgumentException("Goal is invalid or in collision.");

        m_nodes.Clear();
        m_nodes.Add(new Node(start, -1));

        for (int it = 0; it < m_maxIterations; it++)
        {
            double[] qRand = (m_rng.NextDouble() < m_goalSampleRate) ? goal : SampleRandom();
            int iNear = NearestIndex(qRand);
            double[] qNew = Steer(m_nodes[iNear].Q, qRand, m_stepSize);

            if (!InBounds(qNew)) continue;
            if (!IsSegmentFree(m_nodes[iNear].Q, qNew)) continue;

            m_nodes.Add(new Node(qNew, iNear));
            int iNew = m_nodes.Count - 1;

            if (Dist(qNew, goal) <= m_goalThreshold && IsSegmentFree(qNew, goal))
            {
                m_nodes.Add(new Node(goal, iNew));
                var path = Reconstruct(m_nodes.Count - 1);
                if (m_smooth) Smooth(path, m_smoothIters);
                return path;
            }
        }

        // Fallback: best node toward goal
        if (m_nodes.Count > 0)
        {
            int best = NearestIndex(goal);
            var path = Reconstruct(best);
            if (m_smooth) Smooth(path, m_smoothIters);
            return path;
        }
        return null;
    }

    // Helper functions
    private bool InBounds(double[] q)
    {
        for (int i = 0; i < m_n; i++)
            if (q[i] < m_min[i] || q[i] > m_max[i]) return false;
        return true;
    }

    private double[] SampleRandom()
    {
        var q = new double[m_n];
        for (int i = 0; i < m_n; i++)
            q[i] = m_min[i] + m_rng.NextDouble() * (m_max[i] - m_min[i]);
        return q;
    }

    private int NearestIndex(double[] q)
    {
        int best = -1;
        double bestD2 = double.MaxValue;
        for (int i = 0; i < m_nodes.Count; i++)
        {
            double d2 = Dist2(m_nodes[i].Q, q);
            if (d2 < bestD2) { bestD2 = d2; best = i; }
        }
        return best;
    }

    private static double[] Steer(double[] from, double[] to, double step)
    {
        double[] dir = Sub(to, from);
        double norm = Norm(dir);
        if (norm <= step || norm < 1e-12) return (double[])to.Clone();
        double scale = step / norm;
        var q = new double[from.Length];
        for (int i = 0; i < q.Length; i++) q[i] = from[i] + dir[i] * scale;
        return q;
    }

    private bool IsSegmentFree(double[] a, double[] b)
    {
        if (m_isSegmentFreeOpt != null) return m_isSegmentFreeOpt(a, b);

        double L = Dist(a, b);
        int steps = Math.Max(1, (int)Math.Ceiling(L / m_segmentResolution));
        for (int k = 0; k <= steps; k++)
        {
            double t = (double)k / steps;
            var q = Lerp(a, b, t);
            if (!InBounds(q) || !m_isFree(q)) return false;
        }
        return true;
    }

    private List<double[]> Reconstruct(int idx)
    {
        var path = new List<double[]>();
        int cur = idx;
        while (cur >= 0)
        {
            path.Add((double[])m_nodes[cur].Q.Clone());
            cur = m_nodes[cur].Parent;
        }
        path.Reverse();
        return path;
    }

    private void Smooth(List<double[]> path, int iters)
    {
        if (path == null || path.Count < 3) return;
        for (int k = 0; k < iters; k++)
        {
            if (path.Count < 3) break;
            int i = m_rng.Next(0, path.Count - 2);
            int j = m_rng.Next(i + 2, path.Count);
            var a = path[i];
            var b = path[j];
            if (IsSegmentFree(a, b))
            {
                path.RemoveRange(i + 1, j - i - 1);
            }
        }
    }

    // Vector ops
    private static double[] Sub(double[] a, double[] b)
    {
        var r = new double[a.Length];
        for (int i = 0; i < a.Length; i++) r[i] = a[i] - b[i];
        return r;
    }
    private static double Dist2(double[] a, double[] b)
    {
        double s = 0;
        for (int i = 0; i < a.Length; i++) { double d = a[i] - b[i]; s += d * d; }
        return s;
    }
    private static double Dist(double[] a, double[] b) => Math.Sqrt(Dist2(a, b));
    private static double Norm(double[] a) { double s = 0; for (int i = 0; i < a.Length; i++) s += a[i] * a[i]; return Math.Sqrt(s); }
    private static double[] Lerp(double[] a, double[] b, double t)
    {
        var r = new double[a.Length];
        for (int i = 0; i < a.Length; i++) r[i] = a[i] + (b[i] - a[i]) * t;
        return r;
    }
}


} // namespace Trajectory_Planner_Augmentus