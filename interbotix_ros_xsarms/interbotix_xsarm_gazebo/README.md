# interbotix_xsarm_gazebo

## Overview
This package contains the necessary config files to get any of the many Interbotix X-Series arms simulated in Gazebo. Specifically, it contains the [interbotix_texture.gazebo](config/interbotix_texture.gazebo) file which allows the black texture of the robotic arms to display properly (following the method explained [here](http://answers.gazebosim.org/question/16280/how-to-use-custom-textures-on-urdf-models-in-gazebo/)). It also contains YAML files with tuned PID gains for the arm and gripper joints so that ros_control can control the arms effectively. This package has one of two applications. It can either be used in conjunction with MoveIt via the FollowJointTrajectory interface or it can be used by itself via the JointPositionController interface.

## Structure
![xsarm_gazebo_flowchart](images/xsarm_gazebo_flowchart.png)
As shown above, the *interbotix_xsarm_gazebo* package builds on top of the *interbotix_xsarm_descriptions* and *gazebo_ros* packages. To get familiar with the nodes in the *interbotix_xsarm_descriptions* package, please look at its README. The other nodes are described below:
- **gzserver** - responsible for running the physics update-loop and sensor data generation
- **gzclient** - provides a nice GUI to visualize the robot simulation
- **controller_manager** - responsible for loading and starting a set of controllers at once, as well as automatically stopping and unloading those same controllers
- **spawn_model** - adds the robot model as defined in the 'robot_description' parameter into the Gazebo world

## Usage
To run this package, type the line below in a terminal (assuming the WidowX 250 is being launched with trajectory controllers). Note that if you're using a 4 or 6 dof arm, you should set the `dof` launch file argument appropriately.
```
$ roslaunch interbotix_xsarm_gazebo xsarm_gazebo.launch robot_model:=wx250 use_trajectory_controllers:=true
```

Since by default, Gazebo is started in a 'paused' state (this is done to give time for the controllers to kick in), unpause the physics once it is fully loaded by typing:
```
$ rosservice call /gazebo/unpause_physics
```
This is the bare minimum needed to get up and running. Take a look at the table below to see how to further customize with other launch file arguments.

| Argument | Description | Default Value |
| -------- | ----------- | :-----------: |
| robot_model | model type of the Interbotix Arm such as 'wx200' or 'rx150' | "" |
| robot_name | name of the robot (typically equal to `robot_model`, but could be anything) | "$(arg robot_model)" |
| base_link_frame | name of the 'root' link on the arm; typically 'base_link', but can be changed if attaching the arm to a mobile base that already has a 'base_link' frame| 'base_link' |
| show_ar_tag | if true, the AR tag mount is included in the 'robot_description' parameter; if false, it is left out; set to true if using the AR tag mount in your project | false |
| show_gripper_bar | if true, the gripper_bar link is included in the 'robot_description' parameter; if false, the gripper_bar and finger links are not loaded to the parameter server. Set to false if you have a custom gripper attachment | true |
| show_gripper_fingers | if true, the gripper fingers are included in the 'robot_description' parameter; if false, the gripper finger links are not loaded to the parameter server. Set to false if you have custom gripper fingers | true |
| use_world_frame | set this to true if you would like to load a 'world' frame to the 'robot_description' parameter which is located exactly at the 'base_link' frame of the robot; if using multiple robots or if you would like to attach the 'base_link' frame of the robot to a different frame, set this to false | true |  
| external_urdf_loc | the file path to the custom urdf.xacro file that you would like to include in the Interbotix robot's urdf.xacro file| "" |
| use_rviz | launches Rviz | true |
| world_name | the file path to the Gazebo 'world' file to load | refer to [xsarm_gazebo.launch](launch/xsarm_gazebo.launch) |
| gui | launch the Gazebo GUI | true |
| debug | Start gzserver in debug mode using gdb | false |
| paused | start Gazebo in a paused state | true |
| recording | enable Gazebo state log recording | false |
| use_sim_time | tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic /clock | true |
| use_position_controllers | set to true to have the ability to command arbitrary positions to the arm joints in Gazebo | false |
| use_trajectory_controllers | set to true to have the ability to command joint trajectories to the arm joints in Gazebo | false |
| dof | degrees of freedom of the specified arm (only necessary if *use_position_controllers* is set to true) | 5 |
