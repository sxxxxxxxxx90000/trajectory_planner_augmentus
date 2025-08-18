using UnityEngine;

namespace Trajectory_Planner_Augmentus
{
    public class UnityRunner : MonoBehaviour
    {
        private PathPlanner path_planner = new PathPlanner();
        private TrajectoryParameterizer trajectory_parameterizer = new TrajectoryParameterizer();

        // Called once when the script starts
        void Start()
        {
            Debug.Log("Start the UnityRunner");
            // 1. Run path planner

            // 2. Run time parameterization
        }

        // Called every frame
        void Update()
        {
            // Update the frame
        }
    }



} // namespace Trajectory_Planner_Augmentus