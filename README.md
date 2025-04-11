# Jackal UGV Control (ROS 1) using Teleop
A ROS 1 package for teleoperating the Clearpath Jackal UGV using keyboard input. Velocities are smoothly ramped. The teleop node publishes velocity commands to `/jackal_velocity_controller/cmd_vel`. This node uses termios for capturing keyboard input — it must be run in a terminal.

![](jackal.gif)

## Package Name
`jacky_control`
### Features

- Classic 9-key keyboard layout (`u`, `i`, `o`, `j`, `k`, `l`, `m`, `,`, `.`)
- Publishes to `/jackal_velocity_controller/cmd_vel`
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

##  Setup

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
   `$ catkin_make --pkg jacky_control`<br/>
   `$ source devel/setup.bash`
   - Launch the Simulated Jackal in a simple example world:<br/>
   `$ roslaunch jackal_gazebo jackal_world.launch`
   - In another terminal, launch the teleop node:<br/>
   `roslaunch jacky_control jackal_teleop_key.launch`
   - You may alternatively run the node directly:<br/>
   `rosrun jacky_control jackal_teleop_key.py`

### License

This project includes code originally licensed under the [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause).<br/>

© 2011 Willow Garage, Inc. — Modified by Siddhant Baroth, 2025

### Author:
❄️Siddhant Baroth, 2025
