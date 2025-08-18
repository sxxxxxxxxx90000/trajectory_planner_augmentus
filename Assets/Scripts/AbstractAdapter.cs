using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{

public abstract class AbstractAdapter
{
    // Abstract interface for the planner adapter
    public abstract List<double[]> Plan(double[] start, double[] goal);
}

} // namespace Trajectory_Planner_Augmentus