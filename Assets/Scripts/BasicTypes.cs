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

        public Point(double[] pt)
        {
            foreach (double dim_v in pt)
            {
                dims.Add(dim_v);
            }
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

        public Path(List<double[]> pts)
        {
            foreach (double[] pt in pts)
            {
                points.Add(new Point(pt));
            }
        }

        public List<double[]> convertToListArray()
        {
            List<double[]> listArray = new List<double[]>();
            foreach (Point pt in points)
            {
                listArray.Add(pt.convertToArray());
            }
            return listArray;
        }
    }

    public class TrajectoryPoint
    {
        public double Time;
        public double[] Pos;
        public double[] Vel;

        public TrajectoryPoint(double time, double[] pos, double[] vel)
        {
            Time = time;
            Pos = pos;
            Vel = vel;
        }

        public TrajectoryPoint(TrajectoryPoint pt)
        {
            Time = pt.Time;
            Pos = pt.Pos;
            Vel = pt.Vel;
        }
    }

    public class Trajectory
    {
        public List<TrajectoryPoint> trajectory_points;
        public int count = 0;
        public double time_interval = 1;

        public Trajectory(List<TrajectoryPoint> points, double t)
        {
            trajectory_points = points;
            count = points.Count;
            time_interval = t;
        }

        public Trajectory(Trajectory tj)
        {
            trajectory_points = new List<TrajectoryPoint>(tj.trajectory_points);
            count = tj.count;
            time_interval = tj.time_interval;
        }
    }

} // namespace Trajectory_Planner_Augmentus