
# ZJUT-Deus-Nav

`ZJUT-Deus-Nav` is a ROS-based navigation and strategy control system for an autonomous sentry robot, designed for the Zhejiang University of Technology (ZJUT) team. The core of this project is a **Behavior Tree** that enables the robot to make intelligent, real-time decisions by managing complex states like patrolling, chasing enemies, attacking, and retreating.

## ✨ Project Features

- **ROS-Based Architecture**: Fully integrated with the Robot Operating System (ROS) for modularity and communication.
- **Intelligent Strategy Control**: Uses **Behavior Trees** (`py_trees`) to manage and prioritize robot actions in a clear and scalable way.
- **Dynamic Behaviors**: Implements key strategies for an autonomous sentry:
  - **Escape Logic**: Retreats to a safe point when health is low.
  - **Chase Logic**: Actively pursues designated enemy targets.
  - **Attack Logic**: Advances to a strategic point when conditions are favorable.
  - **Patrol Logic**: Patrols a predefined area when no other actions are required.
- **Hardware Communication**: Interfaces with lower-level hardware (e.g., motor controllers) via serial (UART) communication.
- **Custom ROS Messages**: Defines custom messages like `ChaseCommand.msg` for clear communication between strategy components.

## 🤖 Core Architecture: The Behavior Tree

The robot's decision-making is managed by a behavior tree. The tree evaluates conditions and executes actions based on a defined priority order.

**Priority Order:**

1. **`Escape`**: If health is low, retreat to the home position.
2. **`Chase`**: If a chase command is active, pursue the enemy.
3. **`Attack`**: If health is full, advance to the center attack point.
4. **`Patrol`**: If none of the above conditions are met, patrol the area.

This structure is implemented in `communication_BT.py` using the `py_trees` library.

## 🛠️ Technical Stack

- **Framework**: ROS (Robot Operating System)
- **Language**: Python
- **Libraries**: `py_trees`, `py_trees_ros`
- **Communication Protocol**: Serial (UART) for low-level control

## 📦 Installation and Setup

This project is a ROS package. Ensure you have a working ROS environment (e.g., ROS Noetic).

1. **Clone the Repository**:
   Clone this repository into the `src` folder of your Catkin workspace.

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Beatrice-kk/ZJUT-Deus-Nav.git
   ```
2. **Install Dependencies**:
   This project requires `py_trees` and `py_trees_ros`. If not installed, you can install them via pip:

   ```bash
   pip install py_trees py_trees_ros
   ```
3. **Build the Package**:
   Build your Catkin workspace to make the package available in your ROS environment.

   ```bash
   cd ~/catkin_ws
   catkin_make
   # or 'catkin build'
   ```

## 🚀 How to Run

1. **Source your Workspace**:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```
2. **Run the Main Node**:
   Launch the main strategy node using `rosrun`. This script starts the behavior tree and begins controlling the robot.

   ```bash
   rosrun Deus_Sentry_Nav communication_BT.py
   ```

## 📁 Directory Structure

```
ZJUT-Deus-Nav/
├── package.xml            # ROS package manifest
├── CMakeLists.txt         # ROS build file
├── src/
│   └── Deus_Sentry_Nav/
│       ├── msg/
│       │   └── ChaseCommand.msg   # Custom message for chase commands
│       └── src/
│           └── strategy/
│               ├── scripts/
│               │   └── communication_BT.py  # Main script with Behavior Tree logic
│               └── ...
└── README.md              # Project documentation
```

## 🤝 Contribution

Feel free to submit issues or pull requests to improve the robot's functionality and strategies. All contributions are welcome!

## 📄 License

This project is currently not under a specific license. Please contact the author before use.
