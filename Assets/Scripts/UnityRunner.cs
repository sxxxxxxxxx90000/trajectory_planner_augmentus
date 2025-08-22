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


        private Point m_start;
        private Point m_end;
        private double m_sampleInterval = -1;
        private double m_desiredSpeed = -1;
        private double m_maxAcc = -1;
        private double m_maxDec = -1;

        // Default params for 
        public double DEFAULT_SAMPLE_INTERVAL = 0.01;
        public double DEFAULT_DESIRED_SPEED = 1;
        public double DEFAULT_MAX_ACC = 1;
        public double DEFAULT_MAX_DEC = 1; 

        void Awake() { 
            m_object = GetComponent<Rigidbody>(); 
            if (m_object == null)
                Debug.LogError("[UnityRunner] Rigidbody missing on the same GameObject.", this);
        }

        // Called once when the script starts
        void Start()
        {
            Debug.Log("Start the UnityRunner");
            
            // 0. Get start, end, sample interval, desired speed, and acceleration/deceleration, use default otherwise
            if (m_start == null)
            {
                double[] start_pos = {0, 0, 0};
                m_start = new Point(start_pos);
            }
            if (m_end == null)
            {
                double[] end_pos = {10, 10, 10};
                m_end = new Point(end_pos);
            }
            if (m_sampleInterval == -1)
            {
                m_sampleInterval = DEFAULT_SAMPLE_INTERVAL;
            }
            if (m_desiredSpeed == -1)
            {
                m_desiredSpeed = DEFAULT_DESIRED_SPEED;
            }
            if (m_maxAcc == -1)
            {
                m_maxAcc = DEFAULT_MAX_ACC;
            }
            if (m_maxDec == -1)
            {
                m_maxDec = DEFAULT_MAX_DEC;
            }

            // 1. Run path planner
            Debug.Log("Path Planner is started");
            m_pathPlanner.AddStartEndPoints(m_start, m_end);
            m_pathPlanner.Process();
            m_path = m_pathPlanner.GetResult();
            Debug.Log("Path Planner is finished, " + m_path.points.Count + " points in the path");

            // 2. Run trajectory optimizer
            Debug.Log("Trajectory Optimizer is started");
            m_trajectoryOptimizer.SetSampleInterval(m_sampleInterval);
            m_trajectoryOptimizer.SetDesiredSpeed(m_desiredSpeed);
            m_trajectoryOptimizer.SetAccelerationAndDeceleration(m_maxAcc, m_maxDec);
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

        // APIs for users setting the inputs for the planner
        public void SetStartEndPosition(double[] start, double[] end)
        {
            m_start = new Point(start);
            m_end = new Point(end);
        }

        public void SetSampleInterval(double sampleInterval)
        {
            m_sampleInterval = sampleInterval;
        }

        public void SetDesiredSpeed(double desiredSpeed)
        {
            m_desiredSpeed = desiredSpeed;
        }

        public void SetAccelerationAndDeceleration(double acc, double dec)
        {
            m_maxAcc = acc;
            m_maxDec = dec;
        }
    }


} // namespace Trajectory_Planner_Augmentus