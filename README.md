# Jackal UGV Control (ROS 2 Foxy) using Teleop

A ROS 2 package for teleoperating the Clearpath Jackal UGV using keyboard input. Velocities are smoothly ramped. The teleop node publishes velocity commands to `/jackal_velocity_controller/cmd_vel`. This node uses `termios` for capturing keyboard input — it must be run in a terminal or inside a terminal emulator like `xterm`.

![](jackal.gif)

## Package Name

`jacky_control_ros2`

### Features

- Classic 9-key keyboard layout (`u`, `i`, `o`, `j`, `k`, `l`, `m`, `,`, `.`)
- Publishes to `/jackal_velocity_controller/cmd_vel`
- Supports smooth velocity ramping
- Compatible with ROS 1 Jackal UGV via `ros1_bridge`


### Dependencies

ROS 2 packages:
- `rclpy`
- `geometry_msgs`
- `launch_ros`

ROS 1–2 bridging:
- `ros1_bridge` (dynamic bridge)

## Setup

### Prerequisites

- ROS 1 Noetic and ROS 2 Foxy installed and sourced
- Jackal UGV Simulator (ROS 1) installed: [Setup Instructions](https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html)
- `ros1_bridge` built from source (for dynamic bridging)


###  Simulation
#### 1. Build the package

```bash
cd ~/swarm_ws  # or your ROS 2 workspace
colcon build --symlink-install --packages-select jacky_control_ros2
source install/setup.bash
```
Make the python node an executable. Here, the executable is not contained within a `scripts` folder. You may customize it to be that way, however, be sure to make the respective changes in the `setup.py` file within the package directory. 
```bash
chmod +x src/jacky_control_ros2/jacky_control_ros2/jackal_teleop_key.py
```
#### 2. Running the Simulation
**Step 1: Launch the ROS1 Jackal UGV Simulator** 
```bash
# Terminal 1 (ROS 1)
source /opt/ros/noetic/setup.bash
roscore
```
Open another terminal, and in that...
```bash
# Terminal 2 (still ROS 1)
source /opt/ros/noetic/setup.bash
roslaunch jackal_gazebo jackal_world.launch
```
**Step 2: Bring-up the ROS bridge**
```bash
# Terminal 3 - here source both ROS installations
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge
```
**Step 3: Run the ROS 2 Teleop Node**
```bash
# Terminal 4 (ROS 2)
source ~/swarm_ws/install/setup.bash
ros2 run jacky_control_ros2 jackal_teleop_key
```
This node captures key presses via `termios`. It must run in a TTY terminal — not via ros2 launch, unless wrapped with xterm -e.
If you wish to use `ros2 launch ...` then install `xterm` package using `sudo apt install xterm`

And there you have it! You can now control the Jackal UGV using the keyboard in ROS2. 
