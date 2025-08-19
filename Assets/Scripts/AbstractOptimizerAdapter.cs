using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{

public abstract class AbstractOptimizerAdapter
{
    // Abstract interface for the optimizer adapter
    public abstract List<TrajectoryPoint> Plan(List<double[]> path, double dt, double vMax, double aMax);
}

} // namespace Trajectory_Planner_Augmentus