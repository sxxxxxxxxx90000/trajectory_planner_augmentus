using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{
    // Point in N-dimensional space 
    public class Point
    {
        public List<double> dims = new List<double>();

        public double[] convertToArray()
        {
            return dims.ToArray();
        }
    }
    
    public class Path
    {
        public List<Point> points = new List<Point>();
    }

    public class Trajectory
    {
        public List<Point> positions = new List<Point>();
        public List<Point> velocities = new List<Point>();
        public List<Point> accelerations = new List<Point>();
        public int Count = 0;
        public double time_interval = 1;
    }

} // namespace Trajectory_Planner_Augmentus