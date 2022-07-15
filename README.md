# Info
Mobile robot navigation using the ROS Navigation Stack

Robot Used: Robotnik Summit-XL (Simulation in Gazebo)

Features: 
- Mapping - Simultaneous Localization and Mapping (SLAM)
- Localization - Adaptive Monte-Carlo Localization (AMCL)
- Path Palnning
- Obstacle Avoidance
- Location Recording
- Autonomous Navigation to registered locations


# Usage
### 1. Open the project ROSDS
Open the online ROS development environment in your browser by following [this link](https://app.theconstructsim.com/#/l/4e7b06f8/)

### 2. Launch the Simulation and Main Program
Open a web shell and launch the program using

`source ~/catkin_ws/devel/setup.bash`

`roslaunch summit_navigation main.launch`

This will start the Gazebo Simulation in window and launch the navigation program. This also launches **RVIZ**, **rqt_robot_steering**.

### 3. Localize the Robot
Open the **RVIZ** window and localize the robot by driving it around for few seconds. Use the **rqt_robot_steering** GUI to drive the robot.

The red _particle cloud_ spread will reduce as the confidence of the robot's position increases. Stop driving once this converges to a small area around the robot.

### 4. Register locations you'll like to command the robot to drive to
_Skip to step 5 to use pre-registered spots_

- Send a navigation goal to the robot using the **2d Nav Goal** tool from the **RVIZ** GUI.
- Open a new web shell and register that location using the service call:

`source ~/catkin_ws/devel/setup.bash`

`rosservice call /record_spot "label: '<location_name>'"`

<location_name> is the name you'll like to register that spot with
Send more navigation goals and record as many spots as you want.

- When you're done, end the spot recording operation by using:

`rosservice call /record_spot "label: 'end'"`

### 5. Autonomously Navigate Summit to pre-registered spots
You can now send the robot to any of the pre-registered spot using:

`rosservice call /goto_loc "label: '<registered_name>'"`

The follwing location_names are already registered:
- Table
- Turtle
- Room
- South_Pole
- Corridor
- Table
- Nort_Pole

_Enjoy!_

Note: There is an issue with the program wrongly perceiving itself as stuck at doors. If this happens, just drive the bot forward/backward using the **rqt_robot_steering** gui for ~2 seconds. It should recover. If it doesn't, the `move_base` node gave up, just re-enter the command.


# Implementation
See [SUMMIT_NAVIGATION.md](https://github.com/danieladejumo17/summit-robot-navigation/blob/main/SUMMIT_NAVIGATION.md) for implementation procedure.

