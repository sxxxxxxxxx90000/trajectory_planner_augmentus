## UML Class Diagram

+---------------------------+           +------------------------------+
|        PathPlanner        | uses ---> |     AbstractPlannerAdapter   |
|---------------------------|           |------------------------------|
| - m_path : Path           |           | + Plan(start[], goal[])      |
| - m_start : Point         |           +--------------^---------------+
| - m_end : Point           |                          |
| - m_plannerAdapter        |                          | implements
| - m_availabilityChecker   |                          |
|---------------------------|           +------------------------------+
| + AddStartEndPoints()     |           |      RRTPlannerAdapter       |
| + Process()               |           |------------------------------|
| + GetResult() : Path      |           | - m_isAvailable : Func<double[],bool> 
+--------------+------------+           | + Plan(...)                  |
               | creates                 +------------------------------+
               v
         +-----------+
         |   Path    |<----------------------+
         |-----------|                       |
         | + points  | (List<Point>)         | reads
         | + convertToListArray()            |
         +-----------+                       |
                 ^                           |
                 | contains                  |
             +-------+                       |
             | Point |                       |
             +-------+                       |

+---------------------------+           +------------------------------+
|    TrajectoryOptimizer    | uses ---> |   AbstractOptimizerAdapter   |
|---------------------------|           |------------------------------|
| - m_path : Path           |           | + Plan(path[], dt, vMax,     |
| - m_trajectory : Trajectory|          |        aMax, dMax)           |
| - m_sample_interval : double          +--------------^---------------+
| - m_desiredSpeed : double                              |
| - m_maxAcc : double                                    | implements
| - m_optimizerAdapter                                    |
|---------------------------|           +------------------------------+
| + SetSampleInterval()     |           |  TrapezoidalOptimizerAdapter |
| + SetPath()               |           |------------------------------|
| + SetDesiredSpeed()       |           | + Plan(...)                  |
| + Process()               |           +------------------------------+
| + GetResult(): Trajectory |
+--------------+------------+
               | creates
               v
        +---------------+
        |  Trajectory   |
        |---------------|
        | + trajectory_points : List<TrajectoryPoint> |
        | + count : int                                 |
        +---------------+
               ^
               | contains
     +---------------------+
     |  TrajectoryPoint    |
     |---------------------|
     | + Time : double     |
     | + Pos  : double[]   |
     | + Vel  : double[]   |
     +---------------------+

+---------------------+        controls        +-----------+
|     UnityRunner     |----------------------->| Rigidbody |
|---------------------|                         +-----------+
| - m_pathPlanner     | orchestrates
| - m_trajectoryOpt   |
| - m_path : Path     |
| - m_traj : Trajectory|
| - m_timeScale : float|
|---------------------|
| + SetStartEndPosition()         converts
| + SetSampleInterval()     +------------------+
| + SetDesiredSpeed()       |  Vector3 (Unity) |
| + convertPointToVec3D() --+------------------+
| (MonoBehaviour)           ^
+---------------------------+
