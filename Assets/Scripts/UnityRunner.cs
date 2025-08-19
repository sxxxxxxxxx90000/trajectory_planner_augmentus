using System.Collections.Generic;
using UnityEngine;

namespace Trajectory_Planner_Augmentus
{
    public class UnityRunner : MonoBehaviour
    {
        private PathPlanner m_pathPlanner = new PathPlanner();
        private TrajectoryOptimizer m_trajectoryOptimizer = new TrajectoryOptimizer();
        private Path m_path;
        private Trajectory m_traj;
        [SerializeField] Rigidbody m_object;
        private int m_step = 0;

        public Point m_start;
        public Point m_end;


        // Called once when the script starts
        void Start()
        {
            Debug.Log("Start the UnityRunner");
            // 0. Get start and end, use default otherwise
            m_start.dims = new List<double>{0, 0, 0};
            m_end.dims = new List<double>{10, 10, 10};

            // 1. Run path planner
            m_pathPlanner.AddStartEndPoints(m_start, m_end);
            m_pathPlanner.Process();
            m_path = m_pathPlanner.GetResult();

            // 2. Run trajectory optimizer
            m_trajectoryOptimizer.SetSampleInterval(DEFAULT_SAMPLE_INTERVAL);
            m_trajectoryOptimizer.SetDesiredSpeed(DEFAULT_DESIRED_SPEED);
            m_trajectoryOptimizer.SetPath(m_path);
            m_trajectoryOptimizer.Process();
            m_traj = m_trajectoryOptimizer.GetResult();
        }

        // Called every frame
        void FixedUpdate()
        {
            if (m_object == null || m_traj == null || m_traj.count == 0 || m_step == m_traj.count) return;

            // Update the frame
            m_object.linearVelocity = convertPointToVec3D(m_traj.trajectory_points[m_step].Vel);
            m_object.MovePosition(convertPointToVec3D(m_traj.trajectory_points[m_step].Pos));
            m_step++;
        }

        // For demo purpose only, assume our trajectory is in 3D Cartesian-space
        public Vector3 convertPointToVec3D(double[] point)
        {
            return new Vector3((float)point[0], (float)point[1], (float)point[2]);
        }


        // Default params for 
        double DEFAULT_SAMPLE_INTERVAL = 0.01;
        double DEFAULT_DESIRED_SPEED = 1;
    }



} // namespace Trajectory_Planner_Augmentus