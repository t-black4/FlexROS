#README
# FlexROS - uArm Robot Control System

This repository contains the implementation for controlling and visualizing the uArm robot using ROS 2 and RViz.

## Prerequisites

- ROS 2 Humble
- uArm Swift Pro robot
- Required ROS 2 packages:
  - [ur_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) (Universal Robots ROS2 Description Package)
  - ur_custom_control

## Installation

1. Clone the repository and required packages:
```bash
mkdir -p ~/flexros_ws/src
cd ~/flexros_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
```

2. Build the workspace:
```bash
cd ~/flexros_ws
colcon build --symlink-install
```

## Getting Started

1. Source your ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

2. Launch RViz with the uArm robot model:
```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5
```

3. Start the robot control node:
```bash
ros2 run ur_custom_control ur_control_node
```

## Moving the Robot

### Using the ur_move Node

1. In a new terminal, run the ur_move node:
```bash
ros2 run ur_custom_control ur_move
```

2. The node accepts movement commands through ROS 2 topics. You can send commands using:
```bash
ros2 topic pub /ur_control/target_pose geometry_msgs/msg/Pose "position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0"
```

## Visualization in RViz

- The robot model will be displayed in RViz
- Real-time joint states and robot position are visualized
- You can interact with the robot model using the interactive markers

## Troubleshooting

1. If the robot doesn't move:
   - Check if all nodes are running
   - Verify the robot is properly connected
   - Check for any error messages in the terminal

2. If RViz doesn't show the robot:
   - Ensure the robot description is properly loaded
   - Check if the TF frames are being published

## Additional Resources

- ROS 2 Documentation: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- uArm Swift Pro Documentation: [https://www.ufactory.cc/](https://www.ufactory.cc/)

source install/setup.bash 
ros2 topic echo /joint_states

ros2 topic echo /ur5/ur_controllers/joint_group_position_controller/state
