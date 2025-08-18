using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{
    // Point in N-dimensional space 
    public class Point
    {
        public List<double> dims;

        public Point(Point point)
        {
            dims = new List<double>(point.dims);
        }

        public double[] convertToArray()
        {
            return dims.ToArray();
        }
    }
    
    public class Path
    {
        public List<Point> points;

        public Path(Path path)
        {
            points = new List<Point>(path.points);
        }
    }

    public class Trajectory
    {
        public List<Point> positions;
        public List<Point> velocities;
        public List<Point> accelerations;
        public int Count = 0;
        public double time_interval = 1;
    }

} // namespace Trajectory_Planner_Augmentus