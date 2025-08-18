

namespace Trajectory_Planner_Augmentus
{
    public class TrajectoryOptimizer
    {
        private Path m_path;
        private Trajectory m_trajectory;
        public float m_sample_interval;

        public void SetSampleInterval(float sample_interval)
        {
            m_sample_interval = sample_interval;
        }

        public void SetPath(Path path)
        {
            m_path = path;
        }

        public void Process() {}

        public Trajectory getResult()
        {
            return m_trajectory;
        }

    }

} // namespace Trajectory_Planner_Augmentus