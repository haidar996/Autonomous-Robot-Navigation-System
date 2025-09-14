# Autonomous-Robot-Navigation-System
A comprehensive ROS-based autonomous robot navigation system featuring multiple path planning algorithms (A*, Dijkstra, APF,RRT), obstacle avoidance, and smooth trajectory control using CSC motion primitives. Includes Gazebo simulation and real-time control.
## 🎯 Features

- 📍 **Multiple Path Planning Algorithms**:
  - A* algorithm with heuristic optimization
  - Dijkstra's algorithm for shortest path
  - Artificial Potential Field (APF) for dynamic obstacle avoidance
- 🌀 **Smooth Motion Control**:
  - CSC (Curvature-Steering-Control) primitives for continuous trajectories
  - Real-time wheel velocity control using differential drive
- 🧠 **Obstacle Detection & Avoidance**
- 🌐 **Gazebo Simulation Integration**
- ⚙️ Configurable via **YAML parameter files**
- 📡 Real-time control and odometry feedback via ROS topics

---

## 🧱 System Architecture

### 📍 Path Planning Module

- **A\*** & **Dijkstra**: Graph-based search with optimality
- **Optimized APF**: Enhanced obstacle handling with potential fields
- **Vertex-Based Obstacle Representation**: Efficient spatial reasoning

### 🧭 Control Module

- **CSC Controller**: Smooth motion with curvature constraints
- **Wheel Velocity Control**: Differential drive logic
- **Trajectory Optimization**: Minimum-time execution

### 🧪 Simulation Environment

- **Gazebo Integration**: Realistic physics and visualization
- **Custom URDF Models**: Robot & obstacle definitions
- **ROS Topics**:
  - `/cmd_vel`: Velocity control
  - `/odom`: Odometry feedback

---

## 🛠️ Installation & Setup

### 🔧 Prerequisites

- ROS (Melodic or newer)
- Python 3
- NumPy
- Matplotlib
- Gazebo Simulator

### 🧱 Building the Package

```bash
cd ~/catkin_ws/src
git clone <repository-url>
cd ..
catkin_make
source devel/setup.bash
🚀 Usage
🔁 Simulation Launch
bash
Copy code
roslaunch p2 1.launch
roslaunch p2 2.launch
roslaunch p2 3.launch
roslaunch p2 4.launch
🧭 Run Navigation Script
bash
Copy code
rosrun p2 final.py
⚙️ Configuration
Customize the param.yaml file:

Initial robot and obstacle positions

Algorithm selection (A* or Dijkstra)

Source and goal coordinates

📁 File Structure
text
Copy code
├── path_planning_part/     # Path planning algorithms
│   ├── A_star.py
│   ├── dijkstra.py
│   ├── APF.py
│   └── optimized_APF.py
├── Control_part/           # Motion control
│   └── CSC.py
├── urdf/                   # Robot models
│   ├── 1.urdf - 4.urdf
│   └── my_robot.xacro
├── launch/                 # Launch files
├── param.yaml              # YAML configuration
└── main_simulation.py      # Main simulation logic
🧠 Algorithms Implemented
Path Planning
A*: Fast heuristic-based search with optimal path guarantee

Dijkstra: Exhaustive search for shortest path

APF: Dynamic field-based obstacle avoidance

Motion Control
CSC (Curvature Steering Control): Smooth, realistic trajectories

Velocity Profiling: Optimized wheel velocity output

✅ Results
Real-time path generation and obstacle avoidance

Minimal overshoot with smooth CSC-based control

Safe and efficient motion within customizable simulated environments

🧩 Applications
🤖 Autonomous indoor/outdoor robots

🏭 Warehouse logistics & automation

🧪 Robotics education and research

🤝 Service and delivery robots
