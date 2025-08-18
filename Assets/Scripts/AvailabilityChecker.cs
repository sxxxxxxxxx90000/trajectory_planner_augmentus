using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{

public class AvailabilityChecker
{
    // Default availability checking function, can be extended to any type of collision checking or sanity checking
    public bool IsAvailable(double[] pos)
    {
        return true;
    }
}

} // namespace Trajectory_Planner_Augmentus