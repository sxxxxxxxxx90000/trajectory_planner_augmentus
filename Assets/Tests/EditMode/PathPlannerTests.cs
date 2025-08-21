// Assets/Tests/EditMode/PathPlannerTests.cs
using System;
using System.Collections.Generic;
using NUnit.Framework;

namespace Trajectory_Planner_Augmentus
{
    
public class PathPlannerTests
{
    [Test]
    public void Process_ReturnsPath_WithStartAndEndPoints()
    {
        // Arrange
        var planner = new PathPlanner(); // defaults to PlannerType.RRT
        double[] start_pos = {0, 0, 0};
        double[] end_pos = {1, 2, 3};
        var start = new Point(start_pos);
        var end = new Point(end_pos);

        planner.AddStartEndPoints(start, end);

        // Act
        planner.Process();
        var path = planner.GetResult();

        // Assert
        Assert.IsNotNull(path, "Path should not be null after Process()");
        Assert.IsNotNull(path.points, "Path.points should be initialized");
        Assert.AreEqual(2, path.points.Count, "Trivial RRT stub should return start & end only");

        CollectionAssert.AreEqual(
            start.convertToArray(),
            path.points[0].convertToArray(),
            "First point should equal start");

        CollectionAssert.AreEqual(
            end.convertToArray(),
            path.points[1].convertToArray(),
            "Second point should equal end");
    }

    [Test]
    public void Constructor_WithUnknownPlannerType_Throws()
    {
        Assert.Throws<ArgumentException>(
            () => new PathPlanner((PlannerType)999),
            "Unknown planner types should throw an ArgumentException");
    }
}


} // namespace Trajectory_Planner_Augmentus
