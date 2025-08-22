using System;
using System.Collections.Generic;

namespace Trajectory_Planner_Augmentus
{
    public enum OptimizerType
    {
        Trapezoidal
    }

    public class TrajectoryOptimizer
    {
        private Path m_path;
        private Trajectory m_trajectory;
        private double m_sample_interval;
        private double m_desiredSpeed;
        private double m_maxAcc = 1;
        private double m_maxDec = 1;
        private AbstractOptimizerAdapter m_optimizerAdapter;

        public TrajectoryOptimizer(OptimizerType type = OptimizerType.Trapezoidal)
        {
            // We only have Trapezoidal optimizer, it can be extended to other types of optimizers
            if (type == OptimizerType.Trapezoidal)
            {
                m_optimizerAdapter = new TrapezoidalOptimizerAdapter();
            }
            else
            {
                throw new ArgumentException("Not a recognized optimizer type");
            }
        }

        public void SetSampleInterval(double sample_interval)
        {
            m_sample_interval = sample_interval;
        }

        public void SetPath(Path path)
        {
            m_path = path;
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

        public void Process() 
        {
            // Run the optimizer algorithm, e.g., Trapezoidal
            var outputTraj = m_optimizerAdapter.Plan(m_path.convertToListArray(), m_sample_interval, m_desiredSpeed, m_maxAcc, m_maxDec);
            m_trajectory = new Trajectory(outputTraj, m_sample_interval);
        }

        public Trajectory GetResult()
        {
            return m_trajectory;
        }

    }

} // namespace Trajectory_Planner_Augmentus