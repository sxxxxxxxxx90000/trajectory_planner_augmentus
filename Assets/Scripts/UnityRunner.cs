using System.Collections.Generic;
using UnityEngine;

namespace Trajectory_Planner_Augmentus
{
    [RequireComponent(typeof(Rigidbody))]
    public class UnityRunner : MonoBehaviour
    {
        private PathPlanner m_pathPlanner = new PathPlanner();
        private TrajectoryOptimizer m_trajectoryOptimizer = new TrajectoryOptimizer();
        private Path m_path;
        private Trajectory m_traj;

        private Rigidbody m_object;
        private int m_step = 0;
        [SerializeField] float m_timeScale = 1f;
        float m_t0;


        public Point m_start;
        public Point m_end;

        void Awake() { 
            m_object = GetComponent<Rigidbody>(); 
            if (m_object == null)
                Debug.LogError("[UnityRunner] Rigidbody missing on the same GameObject.", this);
        }

        // Called once when the script starts
        void Start()
        {
            Debug.Log("Start the UnityRunner");
            
            // 0. Get start and end, use default otherwise
            double[] start_pos = {0, 0, 0};
            double[] end_pos = {10, 10, 10};
            m_start = new Point(start_pos);
            m_end = new Point(end_pos);

            // 1. Run path planner
            Debug.Log("Path Planner is started");
            m_pathPlanner.AddStartEndPoints(m_start, m_end);
            m_pathPlanner.Process();
            m_path = m_pathPlanner.GetResult();
            Debug.Log("Path Planner is finished, " + m_path.points.Count + " points in the path");

            // 2. Run trajectory optimizer
            Debug.Log("Trajectory Optimizer is started");
            m_trajectoryOptimizer.SetSampleInterval(DEFAULT_SAMPLE_INTERVAL);
            m_trajectoryOptimizer.SetDesiredSpeed(DEFAULT_DESIRED_SPEED);
            m_trajectoryOptimizer.SetPath(m_path);
            m_trajectoryOptimizer.Process();
            m_traj = m_trajectoryOptimizer.GetResult();
            Debug.Log("Trajectory Optimizer is finished, " + m_traj.count + " points in the trajectory");

            // Set start time
            m_t0 = Time.fixedTime;
        }

        // Called every frame
        void FixedUpdate()
        {
            if (m_object == null || m_traj == null || m_traj.count == 0 || m_step == m_traj.count) return;

            // Calculate the current time index for trajectory point
            float tNow = (Time.fixedTime - m_t0) * m_timeScale;
            while (m_step + 1 < m_traj.count && m_traj.trajectory_points[m_step + 1].Time <= tNow) m_step++;

            // Update the frame
            m_object.linearVelocity = convertPointToVec3D(m_traj.trajectory_points[m_step].Vel);
            m_object.MovePosition(convertPointToVec3D(m_traj.trajectory_points[m_step].Pos));
            // m_step++;
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