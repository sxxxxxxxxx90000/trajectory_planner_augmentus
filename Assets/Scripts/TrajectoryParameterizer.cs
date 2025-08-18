

namespace Trajectory_Planner_Augmentus
{
    public class TrajectoryParameterizer
    {
        private Path m_path = new Path();
        private Trajectory m_trajectory = new Trajectory();
        public float m_sample_interval = new float();

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