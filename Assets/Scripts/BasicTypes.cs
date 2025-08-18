using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{
    // Point in N-dimensional space 
    public class Point
    {
        public List<double> m_dims = new List<double>();

        public double[] convertToArray()
        {
            return m_dims.ToArray();
        }
    }
    
    public class Path
    {
        public List<Point> m_points = new List<Point>();
    }

    public class Trajectory
    {
        public List<Point> m_positions = new List<Point>();
        public List<Point> m_velocities = new List<Point>();
        public List<Point> m_accelerations = new List<Point>();
        public double time_interval = new double();
    }

} // namespace Trajectory_Planner_Augmentus