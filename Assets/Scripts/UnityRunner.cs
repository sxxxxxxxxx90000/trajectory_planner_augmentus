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
            m_end.dims = new List<double>{1, 1, 1};

            // 1. Run path planner
            m_pathPlanner.AddStartEndPoints(m_start, m_end);
            m_pathPlanner.Process();
            m_path = m_pathPlanner.GetResult();

            // 2. Run trajectory optimizer
        }

        // Called every frame
        void FixedUpdate()
        {
            if (m_object == null || m_traj == null || m_traj.Count == 0 || m_step == m_traj.Count) return;

            // Update the frame
            m_object.linearVelocity = convertPointToVec3D(m_traj.velocities[m_step]);
            m_object.MovePosition(convertPointToVec3D(m_traj.positions[m_step]));
            m_step++;
        }

        // For demo purpose only, assume our trajectory is in 3D Cartesian-space
        public Vector3 convertPointToVec3D(Point point)
        {
            return new Vector3((float)point.dims[0], (float)point.dims[1], (float)point.dims[2]);
        }
    }



} // namespace Trajectory_Planner_Augmentus