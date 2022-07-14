## Start the Gazebo Simulation
`roslaunch summit_xl_gazebo main.launch`


## See what topics are available
`rostopics list`
- /cmd_vel
- /move_base/cmd_vel
---
- /hokuyo_base/scan
- /summit_xl/odom
- /summit_xl_control/cmd_vel


## Generate the Transform Tree
`rosrun tf view_frames`
- base_footprint
- base_link


---

# Mapping
## Create the summit_mapping package
`catkin_create_pkg summit_mapping rospy`


## Create a launch file to start the slam_gmapping node
__catkin_ws/src/summit_mapping/launch/slam.launch__:
```xml
<launch>
    <arg name="base_frame" default="base_footprint" />
    <arg name="scan_topic" default="hokuyo_base/scan" />
    <arg name="odom_frame" default="odom" />

    <node pkg="gmapping" type="slam_gmapping" name="slam" output="screen" >
        <param name="base_frame" value="$(arg base_frame)" />
        <param name="odom_frame" value="$(arg odom_frame)" />
        <rosparam file="$(find summit_mapping)/params/slam_gmapping_params.yml" command="load" />
        <remap from="scan" to="$(arg scan_topic)" />
    </node> 
</launch>
```

__catkin_ws/src/summit_mapping/params/slam_gmapping_params.yml__:
```yaml
# See http://docs.ros.org/en/hydro/api/gmapping/html/ for a full parameter list

# Initial map parameters
xmin: -100    # metres
xmax: 100     # metres
ymin: -100    # metres
ymax: 100     # metres
delta: 0.05   # metres

# mapping
map_frame: "map"
map_update_interval: 1.0  # seconds
# base_frame | odom_frame | scan_topic | throttle_scans
# linearUpdate | angularUpdate

# laser parameters
maxRange: 8.0   # metres
maxURange: 2.0  # metres
minimumScore: 200
```


## Launch RVIZ
`rosrun rviz rviz`


## Launch Keyboard Teleoperation with
`roslaunch summit_xl_gazebo keyboard_teleop.launch`
or
`rosrun rqt_robot_steering rqt_robot_steering`


## Add Mapping Displays to RVIZ
- Change Global Options: Fixed Frame to 'odom'
- Add RobotModel display
- Add a LaserScan display and change the topic to '/hokuyo_base/scan'. Optionally change the style to 'Points'
- Add a Map display and set map topic to '/map'


## Launch slam.launch
`roslanch summit_mapping slam.launch`


## Move the summit robot around using keyboard teleoperation or rqt_robot_steering
Finish mapping the entire environment this way


## Create a maps folder in the summit_mapping package
`mkdir ~/catkin_ws/src/summit_mapping/maps`

## Save the map the the maps folder using the map_server package
`cd ~/catkin_ws/src/summit_mapping/maps`
`rosrun map_server map_saver -f summit_env`


## Create a launch file for the map_server node to provide a static map
__~catkin_ws/src/summit_mapping/launch/map_server.launch__:
```xml
<launch>
    <arg name="map_file" default="$(find summit_mapping)/maps/summit_env.yaml" />
    <node pkg="map_server" type="map_server" name="map_server" output="screen" args="$(arg map_file)" />
</launch>
```

## Stop the SLAM node and launch the map_server node
`roslaunch summit_mapping map_server.launch`


## Get the info about the map to validate that the map_server is providing a map
__~catkin_ws/src/summit_mapping/src/get_map_data.py__:
```python
#! /usr/bin/env python

import rospy
from nav_msgs.srv import GetMap

rospy.init_node("get_map_data")
rospy.loginfo("get_map_data node started")

service = rospy.ServiceProxy("static_map", GetMap)
service.wait_for_service()
rospy.loginfo("/static_map service is ready")

response = service()
rospy.loginfo("Static Map Data:\n- Resolution: {}\n- Dimension: {} x {}".format(
response.map.info.resolution, response.map.info.width, response.map.info.height))
```
`chmod +x ~catkin_ws/src/summit_mapping/src/get_map_data.py`
`rosrun summit_mapping get_map_data.py`
- the map can be visually inspected in RVIZ but the transform will not be accurate yet. No tf data for the map frame anymore, since we stopped slam_gmapping


## Save RVIZ configuration
- Use the file menu of RVIZ to save the current RVIZ configuration to __/home/user/catkin_ws/src/summit_mapping/rviz/mapping.rviz__
Note: Use `Save as` so that you don't overwrite an existing config. file


---


# Localization
## Create the summit_localization package
`catkin_create_pkg summit_localization rospy`


## Create a launch file and parameters for the amcl node
- Include the map_server launch file to provide map data for localization
__~/catkin_ws/src/summit_localization/launch/amcl.launch__:
```xml
<launch>
    <arg name="map_file" default="$(find summit_mapping)/maps/summit_env.yaml" />
    <arg name="scan_topic" default="hokuyo_base/scan" />

    <include file="$(find summit_mapping)/launch/map_server.launch">
        <arg name="map_file" value="$(arg map_file)" />
    </include>

    <node pkg="amcl" type="amcl" name="amcl">
        <rosparam file="$(find summit_localization)/params/amcl_params.yml" command="load" />
        <remap from="scan" to="$(arg scan_topic)" />
    </node>
</launch>
```

__~/catkin_ws/src/summit_localization/params/amcl_params.yml__:
```yaml
# See http://wiki.ros.org/amcl#Parameters for a full parameter list

# -------------------------------------------
# Overall filter parameters
# -------------------------------------------
min_particles: 100
max_particles: 5000
update_min_d: 0.2 # metres
update_min_a: 0.523 # PI/6 radians
initial_pose_x: 0.0 # metres
initial_pose_y: 0.0 # metres
initial_pose_a: 0.0 # radians
use_map_topic: false
first_map_only: false


# -------------------------------------------
# Laser model parameters
# -------------------------------------------
laser_min_range: -1
laser_max_range: -1


# -------------------------------------------
# Odometry model parameters
# -------------------------------------------
odom_model_type: "diff" # "omni", "diff-corrected" or "omni-corrected"
odom_frame_id: "odom"
base_frame_id: "base_footprint"
global_frame_id: "map"
tf_broadcast: true
```


## Launch the amcl node
`roslaunch summit_localization amcl.launch`


## Add Displays to RVIZ
- Change Global Options: Fixed Frame to 'odom'
- Add RobotModel display
- Add a LaserScan display and change the topic to '/hokuyo_base/scan'. Optionally change the style to 'Points'
- Add a Map display and set map topic to '/map'
- Add a PoseArray display and set the topic to '/particlecloud'
- The Global Options: Fixed Frame can equally be set to 'map' now, since we have tf data for 'map'


## Start the keyboard teleoperation or robot steering to move the robot around
`roslaunch summit_xl_gazebo keyboard_teleop.launch`
or
`rosrun rqt_robot_steering rqt_robot_steering`


## Move the robot around to improve position estimate
Keep moving the robot around gradually untill the PoseArray arrows converge to a much smaller area


## Move Summit to three spots in the environment and record their pose data
Record pose data for Summit
- inside the two rooms, and facing the turtle poster in one of the rooms
- outside, adjacent to the table
Use the topic `/amcl_pose` to get the pose of the robot relative to the map

## Save the RVIZ Configuration
- Save the RVIZ configuration to __/home/user/catkin_ws/src/summit_localization/rviz/amcl_localization.rviz__.


---


# Spot Recording
## Create a package that provides a service node to automatically save the robot's spot when called with a spot label
`catkin_create_pkg spot_recorder spot_recorder.launch`


## Check if a standard srv message exists that fits this need, else, create a new service message
```text
string label
---
bool success
string filename
```
`rossrv list | grep std_srvs`
`rossrv show std_srvs/Trigger`
`rossrv show std_srvs/SetBool`


## Create a new service message in the spot_recorder package
__~/catkin_ws/src/spot_recorder/srv/RecordSpot.srv__:
```text
string label
---
bool success
string filename
```
### Update the CMakeLists.txt file of this package
Update the following functions
```
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
# ...
add_service_files(
  FILES
  RecordSpot.srv
)
# ...
generate_messages(
  DEPENDENCIES
  std_msgs
)
```


### Update the package.xml file of this package
Add the following tags
```xml
<build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```


### Compile the service
`cd ~/catkin_ws`
`catkin_make`
`source devel/setup.bash`


### Verify the service is compiled
`rossrv show spot_recorder/RecordSpot`


## Create the Python script for the service code
__~/catkin_ws/src/spot_recorder/src/spots_to_file.py__:
```python
#! /usr/bin/env python

import os
import rospy
from spot_recorder.srv import RecordSpot, RecordSpotResponse
from geometry_msgs.msg import PoseWithCovarianceStamped


class SpotRecorder:
    def __init__(self, filename="./spots.txt"):
        self.spots = []
        self.pose = None
        self.filename = filename

        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                         callback=self.pose_callback, queue_size=1)
        self.service = rospy.Service(
            "record_spot", RecordSpot, self.save_spot_callback)

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def save_spot_callback(self, msg):
        if msg.label == "end":
            self.write_spots_to_file()
            rospy.loginfo("Spots data written to {}".format(self.filename))
            return RecordSpotResponse(success=True, filename=self.filename)

        self.spots.append({"label": msg.label, "pose": self.pose})
        rospy.loginfo("Recorded Spot {}".format(msg.label))
        return RecordSpotResponse(success=True, filename="")

    def write_spots_to_file(self):
        with open(self.filename, mode="w") as f:
            for spot in self.spots:
                f.write("Label: {}\n".format(spot["label"]))
                f.write("Pose:\n- Position:\n  -- x: {}\n  -- y: {}\n".format(
                    spot["pose"].position.x, spot["pose"].position.y, spot["pose"].position.z))
                f.write("- Orientation:\n  -- z: {}\n  -- w: {}\n\n".format(
                    spot["pose"].orientation.z, spot["pose"].orientation.w))
            f.close()
            self.spots = []


if __name__ == "__main__":
    rospy.init_node("spot_recorder")
    rospy.loginfo("Spot Recorder Node started")

    recorder = SpotRecorder(filename="/home/user/catkin_ws/spots.txt")
    rospy.spin()
```


## Create a launch file to start the spot recording node
__~/catkin_ws/src/spot_recorder/launch/spot_recorder.launch__:
```xml
<launch>
    <!-- TODO: Pass argument to save output to -->
    <node pkg="spot_recorder" type="spots_to_file.py" name="spot_recorder" output="screen" />
</launch>
```

## Test it: Launch the spot_recorder node
- Ensure the amcl node is launched first
- Ensure the catkin workspace is sourced in the terminal, so that the service message is available
`roslaunch spot_recorder spot_recorder.launch`

## Test it: Move the robot to a spot to be recorded and call the service
- Ensure the catkin workspace is sourced in the terminal
`rosservice call record_spot "label: 'Room'"`

## Test it: Pass label 'end' to end spot recording and get the saved file
`rosservice call record_spot "label: 'end'"`

---

# Path Planning
## Create the path planning package
`cd ~/catkin_ws/src`
`catkin_create_pkg summit_path_planning rospy`

## Create the move base launch file
__~/catkin_ws/src/summit_path_planning/launch/move_base.launch__:
```xml
<?xml version="1.0"?>
<launch>
    <arg name="scan_topic" default="hokuyo_base/scan" />

    <!-- Launch AMCL -->
    <include file="$(find summit_localization)/launch/amcl.launch">
        <arg name="map_file" value="$(find summit_mapping)/maps/summit_env.yaml" />
        <arg name="scan_topic" value="$(arg scan_topic)" />
    </include>

    <!-- Remap Topics -->
    <remap from="cmd_vel" to="summit_xl_control/cmd_vel" />
    <remap from="scan" to="$(arg scan_topic)" />
    <remap from="odom" to="summit_xl/odom" />

    <!-- Launch move_base -->
    <node pkg="move_base" type="move_base" name="move_base" output="screen">
        <!-- MOVE BASE CONFIG -->
        <!-- Move Base Parameters -->
        <rosparam file="$(find summit_path_planning)/params/move_base_params.yaml" command="load" />


        <!-- COSTMAPS CONFIG -->
        <!-- Common Costmap Parameters -->
        <rosparam file="$(find summit_path_planning)/params/common_costmap_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find summit_path_planning)/params/common_costmap_params.yaml" command="load" ns="global_costmap" />

        <!-- Global Costmap Parameters -->
        <rosparam file="$(find summit_path_planning)/params/global_costmap_params.yaml" command="load" ns="global_costmap" />

        <!-- Local Costmap Parameters -->
        <rosparam file="$(find summit_path_planning)/params/local_costmap_params.yaml" command="load" ns="local_costmap" />


        <!-- PLANNERS CONFIG -->
        <!-- Global Planner Parameters -->
        <rosparam file="$(find summit_path_planning)/params/navfn_planner_params.yaml" command="load" />

        <!-- Local Planner Parameters -->
        <rosparam file="$(find summit_path_planning)/params/dwa_planner_params.yaml" command="load" />
        
    </node>
</launch>
```

### Create the move_base_params file
__~/catkin_ws/src/summit_path_planning/params/move_base_params.yaml__:
```yaml
# Global and Local Planners
## global_planners: "carrot_planner/CarrotPlanner", "global_planner/GlobalPlanner"
base_global_planner: "navfn/NavfnROS"

## local_planners: "base_local_planner/TrajectoryPlannerROS", "eband_local_planner/EBandPlannerROS", "teb_local_planner/TebLocalPlannerROS"
base_local_planner: "dwa_local_planner/DWAPlannerROS"


controller_frequency: 5  # HZ
recovery_behaviour_enabled: true
```

### Create the global planner, navfn_planner_params file
__~/catkin_ws/src/summit_path_planning/params/navfn_planner_params.yaml__:
```yaml
NavfnROS:
  allow_unknown: true # Specifies whether or not to allow navfn to create plans that traverse unknown space.
  default_tolerance: 0.1  # metres - A tolerance on the goal point for the planner.
```

### Create the local planner, dwa_planner_params file
__~/catkin_ws/src/summit_path_planning/params/dwa_planner_params.yaml__:
```yaml
DWAPlannerROS:
  # Robot Configuration Parameters
  acc_lim_x: 2.5  # m/s^2
  acc_lin_y: 0    # m/s^2
  acc_lim_th: 3.2 # rad/s^2

  max_trans_vel: 0.5  # 0.55 # m/s, absolute value
  min_trans_vel: 0.1  # m/s
  max_vel_x: 0.5
  min_vel_x: 0.0  # 0.1
  max_vel_y: 0.0
  min_vel_y: 0.0

  max_rot_vel: 1.0
  min_rot_vel: 0.2  # 0.4

  # Goal Tolerance Parameters
  yaw_goal_tolerance: 0.2 # 0.05
  xy_goal_tolerance: 0.4  # 0.1
  latch_xy_goal_tolerance: false  # newly added, check effect of removing
  
  # Forward Simulation Parameters
  sim_time: 1.7 # seconds
  # sim_granularity: 0.025  # m, The step size, in meters, to take between points on a given trajectory 
  # vx_samples: 6 # default is three, how does it affect
  # vth_samples: 20

  # Trajectory Scoring Parameters
  path_distance_bias: 32
  # goal_distance_bias: 24
  # occdist_scale: 0.01 # The weighting for how much the controller should attempt to avoid obstacles 

  # Oscillation Prevention Parameters
  # oscillation_reset_dist: 0.05

  # Global Plan Parameters
  # prune_plan: true
```

### Create the common_costmap_params file
__~/catkin_ws/src/summit_path_planning/params/common_costmap_params.yaml__:
```yaml
# Robot Parameters
footprint: [[-0.5, -0.33], [-0.5, 0.33], [0.5, 0.33], [0.5, -0.33]]
footprint_padding: 0.01 # 1 cm

# Coordinate frame and tf parameters
robot_base_frame: "base_footprint"
transform_tolerance: 0.5  # delay in seconds for which a tf transform can be missing

# Rate parameters
update_frequency: 4.0
publish_frequency: 4.0  # costmap display frequency

# Map management parameters
resolution: 0.05  # some parameters can be overidden by the static map layer

# TODO: 
obstacle_range: 5.5 # is this parameter valid here - should be in 'obstacle' layer
raytrace_range: 6.0 # is this parameter valid here - should be in 'obstacle' layer

# Layers
static: # http://wiki.ros.org/costmap_2d/hydro/staticmap
  map_topic: "/map"
  subscribe_to_updates: true
  track_unknown_space: true # pass unknown space values as they are, otherwise, they are translated as FREE_SPACE

obstacles_laser: # http://wiki.ros.org/costmap_2d/hydro/obstacles
  observation_sources: laser  # space separated list of namespaces for observation sources - defined below.
  obstacle_range: 7.0 # max dist. at which to insert an obstacle. Can also be defined per source
  raytrace_range: 7.0 # max dist. at which to remove an obstacle. can also be defined per source
  track_unknown_space: true # pass unknown space values as they are, otherwise, they are translated as FREE_SPACE
  laser: {data_type: LaserScan, clearing: true, marking: true, topic: "scan", inf_is_valid: true}

inflation:  # http://wiki.ros.org/costmap_2d/hydro/inflation
  inflation_radius: 0.20
```

### Create the global_costmap_params file
__~/catkin_ws/src/summit_path_planning/params/global_costmap_params.yaml__:
```yaml
# Coordinate and tf frames
global_frame: "map"

# Map management parameters
rolling_window: false
track_unknown_space: true  # is this parameter valid here - should be in 'static', 'obstacle' layers
static_map: true

# Rate Parameters
update_frequency: 5.0

# Plugins
plugins:
  - {name: static, type: "costmap_2d::StaticLayer"}
  - {name: obstacles_laser, type: "costmap_2d::VoxelLayer"}
  - {name: inflation, type: "costmap_2d::InflationLayer"}
```

### Create the local_costmap_params file
__~/catkin_ws/src/summit_path_planning/params/local_costmap_params.yaml__:
```yaml
# Coordinate and tf frames
global_frame: "odom"

# Map management parameters
rolling_window: true
width: 10
height: 10
# static_map: false

# Rate Parameters
update_frequency: 20

# Plugins
plugins:
  - {name: obstacles_laser, type: "costmap_2d::ObstacleLayer"}
  - {name: inflation, type: "costmap_2d::InflationLayer"}
```

## Launch the move_base node
`roslaunch summit_path_planning move_base.launch`

## Launch rviz (if not launched already)
`rosrun rviz rviz`

## Add the following displays to RVIZ
**Set** Global Options: Fixed Frame = map 
- RobotModel: Robot Description = robot_description
- LaserScan: Topic = /hokuyo_base/scan
- Map: Topic = /map
- PoseArray: Topic = /particlecloud
- Map: Name = Global Costmap, Topic = /move_base/global_costmap/costmap
- Map: Name = Local Costmap, Topic = /move_base/local_costmap/costmap
- Path: Name = Global Plan, Topic = /move_base/NavfnROS/plan, Color = Green
- Path: Name = Local Plan, Topic = /move_base/DWAPlannerROS/local_plan, Color = Red
- Path: Name = DWA Global Plan Portion, Topic = /move_base/DWAPlannerROS/global_plan, Color = Cyan

## Launch the robot steering or keyboard teleoperation (if not already launched)
`rosrun rqt_robot_steering rqt_robot_steering`
or
`roslaunch summit_xl_gazebo keyboard_teleop.launch`

## Move the robot around until it's properly localized
Use the robot steering GUI or keyboard teleoperation to move the robot around until it's properly localized

## Send a goal to the move_base node
Use the 2D Nav Goal tool from the RVIZ GUI to send a goal command to the move base node
- The Global Planner will generate a global plan for this goal
- The Local Planner will follow this plan in segments and move the robot to the goal
- Send more goals!

## Save the RVIZ Configuration to ``
- Save the RVIZ configuration to __/home/user/catkin_ws/src/summit_path_planning/rviz/path_planning.rviz__.


---


# Main Program
A node that provides a service that takes the string label of a spot as request, and moves the robot to that spot using the move_base action server.

## Create the package
`catkin_create_pkg summit_navigation rospy`

## Determine the Service structure to use and see is any such service exist
```text
string label
---
bool success
string message
```
Check if this service exists already
`rossrv list | grep std_srvs`
`rossrv info std_srvs/<service_name>`

## Create the new service in the summit_navigation package
__/home/user/catkin_ws/src/summit_navigation/srv/SummitGoal.srv__:
```text
string label
---
bool success
string message
```

## Modify CMakeLists.txt and package.xml of the summit_navigation package to compile the new service
- Modify the following functions in __/home/user/catkin_ws/src/summit_navigation/CMakeLists.txt__:
```text
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SummitGoal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS rospy
)
```

- Add the following to __/home/user/catkin_ws/src/summit_navigation/package.xml__:
```xml
<package format="2">
...
...
...
  <build_depend>message_generation</build_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
...
...
</package>
```

## Compile the summit_navigation package
`cd ~/catkin_ws`
`catkin_make`

## Verify the service message is compiled
`rossrv show summit_navigation/SummitGoal.srv`
Note: In a new terminal ensure to source the workspace

## Create the service server and the move_base action client
__/home/user/catkin_ws/src/summit_navigation/src/main.py__:
```python
```





----- get the footprint values
----- See if obstacle_range, ... will need to be removed from the root namespace
----- Package dependencies details
