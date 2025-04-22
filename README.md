# Jackal UGV Control (ROS 1) using Teleop
A ROS 1 package for teleoperating the Clearpath Jackal UGV using keyboard input. Velocities are smoothly ramped. The teleop node publishes velocity commands to `/jackalX/jackal_velocity_controller/cmd_vel` for each respective UGV. This node uses termios for capturing keyboard input — it must be run in a terminal.

## Package Name
`multi_jackal_tutorials`
### Features

- Supports controlling 2 or 3 Jackal UGVs simultaneously
- Classic 9-key keyboard layout for each Jackal:
  - `jackal0`: `u`, `i`, `o`, `j`, `k`, `l`, `m`, `,`, `.`
  - `jackal1`: `r`, `t`, `y`, `f`, `g`, `h`, `v`, `b`, `n`
  - `jackal2`: `q`, `w`, `e`, `a`, `s`, `d`, `z`, `x`, `c`
- Publishes to `/jackalX/jackal_velocity_controller/cmd_vel`
- Supports smooth velocity ramping

### Dependencies
The following ROS 1 packages are required to build and use the bridge:
- `rospy`
- `geometry_msgs`
- `catkin`
- `roscpp`
- `roslaunch` (for roscore executable)
- `rosmsg`
- `std_msgs` and `geometry_msgs`
as well as the Python package `rospkg`

## Setup

### Prerequisites:
ROS1 Noetic Installation - http://wiki.ros.org/noetic/Installation <br/>
Jackal UGV Simulator - https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html

### Simulation:

1. #### Running the Simulator<br/>
   - If you follow the link, you'll realize that the simulator isn't a package hosted in a workspace i.e., doesn't have to be cloned.
   - Note: Jackal UGV Simulator can be installed to your home directory.
   - Source ROS1 Environment and start ROS Master
     - `$ source /opt/ros/noetic`
     - `$ roscore`
     - Open a new terminal.
  
2. #### Running the teleop node
   - Clone this repository to your workspace i.e., to: <br/> `$ catkin_ws/src` or the equivalent.
   - Build the workspace:<br/>
     `$ cd ~/path_to_catkin_ws/`<br/>
     `$ catkin_make --pkg multi_jackal_tutorials`<br/>
     `$ source devel/setup.bash`
   - Launch the Simulated Jackals:<br/>
     `$ roslaunch multi_jackal_tutorials three_jackal.launch`
   - In another terminal, launch the teleop node:<br/>
     `rosrun multi_jackal_tutorials multi_jackal_teleop.py`
   - You will be prompted to enter the number of Jackals to control: enter `2` or `3` as required.

### License

This project includes code originally licensed under the [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause).<br/>

© 2011 Willow Garage, Inc. — Modified by Siddhant Baroth, 2025

### Author:
❄️ Siddhant Baroth, 2025
