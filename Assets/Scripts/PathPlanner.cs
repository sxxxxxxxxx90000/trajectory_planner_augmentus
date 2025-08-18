using System;
using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{
    
    public enum PlannerType
    {
        RRT
    }

    public class PathPlanner
    {
        private Path m_path;
        private Point m_start;
        private Point m_end;
        private AbstractAdapter m_plannerAdapter;
        private AvailabilityChecker m_availabilityChecker = new AvailabilityChecker();

        public PathPlanner(PlannerType type = PlannerType.RRT)
        {
            // We only have RRT planner, it can be extended to other types of planners
            if (type == PlannerType.RRT)
            {
                m_plannerAdapter = new RRTAdapter(m_availabilityChecker.IsAvailable);
            }
            else
            {
                throw new ArgumentException("Not a recognized planner type");
            }
        }

        public void AddStartEndPoints(Point start_point, Point end_point)
        {
            m_start = start_point;
            m_end = end_point;
        }

        public void Process() 
        {
            // Run the planner algorithm, e.g., RRT
            // m_path.points = new List<Point>{m_start, m_end}; 
            var outputPath = m_plannerAdapter.Plan(m_start.convertToArray(), m_end.convertToArray());
            m_path = new Path(outputPath);
        }

        public Path GetResult()
        {
            return m_path;
        }

        

    }

} // namespace Trajectory_Planner_Augmentus