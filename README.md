# Trajectory Planner Augmentus

Trajectory Planner Augmentus is a **trajectory planning library for Unity**.  
It provides two core modules which can be used consecutively:

- **Path Planner** — computes a geometric path between start and goal points.
- **Trajectory Optimizer** — time-parameterizes the path into a full trajectory with velocities.

Both modules are able to be used independenly and designed to be **extendable** so other algorithms can be added as well. The architecture is represented in image.

---

## Provided Algorithms

- **Path Planner**: [RRT (Rapidly-exploring Random Tree)] 
- **Trajectory Optimizer**: **Trapezoidal velocity profile**

---

## Typical Code Example
A mini example of how to call the Path Planner and Trajectory Optimizer modules can be found in `Assets/Scripts/UnityRunner.cs`


---


## Setup Instructions

### 1. Unity Setup
1. Open your Unity project.
2. OPen the project folder `Trajectory_Planner_Augmentus`.
3. Ensure the **Test Framework** package (`com.unity.test-framework`) is installed via Package Manager.

### 2. Running the Tests
This project comes with **PlayMode** unit tests.

- Open **Window → Test Runner** (or **Test Framework → Test Runner** in Unity).
- Switch to **Play Mode** tab → Run All.
- ✅ Green means the planner and optimizer modules are working correctly.

### 3. Running the Demo
A simple **3D sphere demo** is provided as a **proof-of-concept**:

1. Open the demo scene in `Assets/3D_sphere_demo.unity`.
2. Press **Play**.
3. A sphere with a Rigidbody sphere will move from a start point to a goal point along a path planned with **RRT** and time-parameterized with the **Trapezoidal optimizer**.

This is a minimal demo, intended to show the system in action. It is **not a polished gameplay feature**.

---

## Next Steps

Planned improvements include:

- **Improve Interfaces**  
  Make the planner/optimizer adapters more flexible and consistent, so adding new algorithms is seamless.

- **Enrich Test Cases**  
  Add more unit and integration tests for corner cases, performance, and multi-dimensional trajectories.

