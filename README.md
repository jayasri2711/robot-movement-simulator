# 🤖 Robot Movement Simulator

A C++-based grid robot simulator demonstrating robot movement, obstacle and boundary detection, simulated sensor scanning, and BFS-based shortest-path planning with automatic navigation.

## 📌 Project Overview

This project simulates a robot navigating a **5 × 5 grid** from a starting position to a predefined goal while avoiding obstacles.

The simulator uses coordinate-based movement, direction control, grid validation, and the Breadth-First Search (BFS) algorithm to plan a shortest valid route through the grid.

> **Project type:** Robotics and Automation — Software Simulation  
> **Language:** C++  
> **Environment:** Console-based simulation

## ✨ Features

- **Grid Navigation:** Robot movement across a 5 × 5 grid.
- **Direction Control:** Forward, backward, left, and right commands.
- **Obstacle Detection:** Checks whether a target cell contains an obstacle.
- **Boundary Detection:** Prevents movement outside the grid.
- **Simulated Sensor Scanning:** Checks front, back, left, and right relative to the robot's orientation.
- **BFS Path Planning:** Calculates a shortest valid path to the goal.
- **Automatic Navigation:** Uses the calculated path to navigate toward the goal.
- **Navigation Statistics:** Tracks movement and collision information.
- **Reset Functionality:** Returns the simulator to its initial state.

## 🗺️ Grid Environment

| Parameter | Value |
|---|---|
| Grid size | 5 × 5 |
| Starting position | `(0, 0)` |
| Goal position | `(4, 4)` |
| Navigation algorithm | Breadth-First Search (BFS) |
| Movement model | One grid cell per successful movement |

The grid contains predefined obstacles. The simulator checks obstacles and grid boundaries before allowing movement.

## 🛠️ Technologies Used

- C++
- Standard Template Library (STL)
- Breadth-First Search (BFS)
- Visual Studio Code
- GCC compiler
- Git and GitHub

## 🚀 Getting Started

### Prerequisites

- A C++ compiler such as GCC
- Visual Studio Code or another code editor (optional)

### 1. Clone the Repository

```bash
git clone https://github.com/jayasri2711/robot-movement-simulator.git
cd robot-movement-simulator
```

### 2. Compile the Program

```bash
g++ main.cpp -o main
```

### 3. Run the Program

**Windows PowerShell:**

```powershell
.\main.exe
```

**Linux:**

```bash
./main
```

## 🎮 Commands

The simulator supports commands for movement, direction control, navigation, and status checking. Available commands are displayed by the program.

Use the exact command names shown in the running simulator.

## 🧪 Testing

The project report documents tests for movement, direction control, obstacle and boundary handling, sensor scanning, BFS path planning, automatic navigation, and reset functionality.

In the documented automatic-navigation test, the robot travelled from `(0, 0)` to `(4, 4)` in **8 successful movements**, with **0 obstacle collisions** and **0 boundary collisions**.

These values describe the documented test run; results may differ when the simulator starts from a different state or receives different commands.

## 📄 Project Report

Read the detailed project documentation:

[**View Robot Movement Simulator (PDF)**](./ROBOT%20MOVEMENT%20SIMULATOR.pdf)

## 🎯 Learning Outcomes

- Applying C++ programming fundamentals to a robotics problem.
- Working with grid coordinates and robot orientation.
- Implementing obstacle and boundary validation.
- Understanding BFS shortest-path planning.
- Combining movement logic, simulated sensing, and automatic navigation.
- Testing and documenting a software-based robotics simulation.

## 🔭 Future Improvements

- Visual grid animation.
- A* path-planning algorithm.
- Integration with ROS 2.
- Connection to a physical robot platform.
- More advanced navigation and sensor models.

These are potential future enhancements, not claims of currently implemented functionality.

## 👩‍💻 Author

**Jayasri.D**

Robotics and Automation Engineering Student
---

*This project is a software simulation for learning and demonstrating fundamental robot-navigation concepts. It does not represent a physical robot implementation.*
