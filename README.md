# Turtlebot3-Path-Planning

Using Robotics Operating System (ROS) Middleware, implemented Breath-First-Search (BFS) algorithm and PID Control in C++ to navigate a TurtleBot3 through a Maze in Gazebo.

**Maze Objective (Goal highlighted in Green)**

![Alt text](imgs/maze-objective.png)

**Optimization of BFS**

Visit Vertex Priority: North, West, South, East. Optimized Path planning algorithm by checking dead ends in adjacent north cells.

![Alt text](imgs/optimization.png)

Optimized BFS Shortest Path to Goal

![Alt text](imgs/optimized-path.png)

**BFS in Action**

![Alt Text](imgs/robot-navigate.gif)
