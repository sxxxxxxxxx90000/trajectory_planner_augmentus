// Assets/Tests/EditMode/TrajectoryOptimizerTests.cs
using System;
using System.Collections.Generic;
using NUnit.Framework;

namespace Trajectory_Planner_Augmentus
{
    
public class TrajectoryOptimizerTests
{
    [Test]
    public void Process_BuildsTrajectory_And_PassesCorrectParameters()
    {
        // Arrange
        var points = new List<double[]>
        {
            new double[]{0, 0, 0},
            new double[]{5, 0, 0},
            new double[]{5, 2, 0}
        };

        var path = new Path(points);

        double dt = 0.02;
        double desiredSpeed = 1.5;

        var opt = new TrajectoryOptimizer();
        opt.SetPath(path);
        opt.SetSampleInterval(dt);
        opt.SetDesiredSpeed(desiredSpeed);

        // Act
        opt.Process();
        var traj = opt.GetResult();

        // Assert: result exists
        Assert.IsNotNull(traj, "Trajectory should not be null after Process().");

        // Trajectory constructed with adapter output & dt
        Assert.AreEqual(dt, traj.time_interval, 1e-12, "Trajectory sample interval mismatch.");

        // Each trajectory point velocity should be equal or less than the desired one
        foreach (var traj_point in traj.trajectory_points)
        {
            Assert.LessOrEqual(Norm(traj_point.Vel), desiredSpeed);
        }

        // Start and end point should match the input and have zero velocity
        var start_traj_pt = traj.trajectory_points[0];
        var end_traj_pt = traj.trajectory_points[traj.count-1];
        Assert.That(start_traj_pt.Pos, Is.EqualTo(new double[]{0, 0, 0}).Within(1e-12));
        Assert.That(end_traj_pt.Pos, Is.EqualTo(new double[]{5, 2, 0}).Within(1e-12));
        Assert.That(start_traj_pt.Vel, Is.EqualTo(new double[]{0, 0, 0}).Within(1e-12));
        Assert.That(end_traj_pt.Vel, Is.EqualTo(new double[]{0, 0, 0}).Within(1e-12));
    }

    [Test]
    public void Constructor_WithUnknownOptimizerType_Throws()
    {
        Assert.Throws<ArgumentException>(
            () => new TrajectoryOptimizer((OptimizerType)999),
            "Unknown optimizer types should throw an ArgumentException.");
    }

    [Test]
    public void Process_WithoutPath_ThrowsNullReference()
    {
        // Depending on how you want to handle this, you might prefer to
        // change TrajectoryOptimizer.Process() to validate inputs and
        // throw a clearer exception. This test documents current behavior.
        var opt = new TrajectoryOptimizer();
        opt.SetSampleInterval(0.01);
        opt.SetDesiredSpeed(1.0);

        Assert.Throws<NullReferenceException>(() => opt.Process(),
            "Calling Process() without SetPath() should currently throw.");
    }

    private static double Norm(double[] v) { double s=0; for(int i=0;i<v.Length;i++) s+=v[i]*v[i]; return Math.Sqrt(s); }
}


}  // namespace Trajectory_Planner_Augmentus
