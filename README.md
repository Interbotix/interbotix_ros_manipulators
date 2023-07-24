![manipulator_banner](images/manipulator_banner.png)

## Overview
![manipulator_repo_structure](images/manipulator_repo_structure.png)
Welcome to the *interbotix_ros_manipulators* repository! This repo contains custom ROS packages to control the various types of arms sold at [Trossen Robotics](https://www.trossenrobotics.com/). These ROS packages build upon the ROS driver nodes found in the [interbotix_ros_core](https://github.com/Interbotix/interbotix_ros_core) repository. Support-level software can be found in the [interbotix_ros_toolboxes](https://github.com/Interbotix/interbotix_ros_toolboxes) repository.

### Build Status

| ROS Distro | X-Series ROS Manipulators Build |
| :------- | :------- |
| ROS 1 Noetic | [![build-xs-noetic](https://github.com/Interbotix/interbotix_ros_manipulators/actions/workflows/xs-noetic.yaml/badge.svg)](https://github.com/Interbotix/interbotix_ros_manipulators/actions/workflows/xs-noetic.yaml) |
| ROS 2 Galactic | [![build-xs-galactic](https://github.com/Interbotix/interbotix_ros_manipulators/actions/workflows/xs-galactic.yaml/badge.svg)](https://github.com/Interbotix/interbotix_ros_manipulators/actions/workflows/xs-galactic.yaml) |
| ROS 2 Humble | [![build-xs-humble](https://github.com/Interbotix/interbotix_ros_manipulators/actions/workflows/xs-humble.yaml/badge.svg)](https://github.com/Interbotix/interbotix_ros_manipulators/actions/workflows/xs-humble.yaml) |
| ROS 2 Rolling | [![build-xs-rolling](https://github.com/Interbotix/interbotix_ros_manipulators/actions/workflows/xs-rolling.yaml/badge.svg)](https://github.com/Interbotix/interbotix_ros_manipulators/actions/workflows/xs-rolling.yaml) |

## Repo Structure

```
GitHub Landing Page: Explains repository structure and contains a single directory for each type of manipulator.
├── Manipulator Type X Landing Page: Contains 'core' arm ROS packages.
│   ├── Manipulator Control ROS Package
│   │   └── Demo Scripts: contains example scripts that build upon the interface modules in the interbotix_ros_toolboxes repository
│   │       ├── Demo Script 1
│   │       ├── Demo Script 2
|   |       └── Demo Script X
│   ├── Core Arm ROS Package 1
│   ├── Core Arm ROS Package 2
│   ├── Core Arm ROS Package X
│   └── Examples: contains 'demo' arm ROS packages that build upon some of the 'core' arm ROS packages
│       ├── Demo Arm ROS Package 1
│       ├── Demo Arm ROS Package 2
│       └── Demo Arm ROS Package X
├── CITATION.cff
├── LICENSE
└── README.md
```

As shown above, there are five main levels to this repository. To clarify some of the terms above, refer to the descriptions below.

- **Manipulator Type** - Any robotic arm that can use the same *interbotix_XXarm_control* package is considered to be of the same type. This division primarily lies on the type of actuator that makes up the robot. As an example, all the X-Series arms are considered the same type of manipulator since they all use DYNAMIXEL X-Series servos despite the fact that they come in different sizes, DOF, and motor versions. However, a robotic arm made up of some other manufacturer's servos, or even half made up of DYNAMIXEL servos and half made up of some other manufacturer's servos would be considered a different manipulator type.

- **Core Arm ROS Package** - This refers to 'High Profile' ROS packages that are essential to make a given arm work. Examples of 'High Profile' ROS packages include:
    - *interbotix_XXarm_descriptions* - contains URDFs and meshes of the arms, making visualization possible
    - *interbotix_XXarm_control* - sets up the proper configurations and makes it possible to control the physical arm
    - *interbotix_XXarm_moveit* - sets up the proper configurations and makes it possible to control an arm via MoveIt
    - *interbotix_XXarm_sim* - sets up the proper configurations and makes it possible to control arms in various simulators (Gazebo, Gazebo Classic, etc.)
    - *interbotix_XXarm_ros_control*  - ROS control package used with MoveIt to control the physical arms

- **Demo Arm ROS Package** - This refers to demo ROS packages that build upon the **Core Arm ROS Packages**. ROS researchers could use these packages as references to learn how to develop their own ROS packages and to get a feel for how the robot works. Typical demos for a given manipulator type include:
    - *interbotix_XXarm_joy* - manipulate an arm's end-effector using a joystick controller
    - *interbotix_XXarm_puppet* - make one or more 'puppet' arms copy the motion of a 'master' arm
    - *interbotix_XXarm_moveit_interface* - learn how to use MoveIt's MoveGroup Python or C++ APIs to control a robot arm

- **Demo Script** - This refers to demo scripts that build upon the interface modules in the *interbotix_ros_toolboxes* repository. These modules essentially abstract away all ROS code, making it easy for a researcher with no ROS experience to interface with an arm as if it was just another object. It also makes sequencing robot motion a piece of cake. These scripts are written in languages that users may feel more comfortable with like Python and MATLAB. The directories that contain demo scripts for each language may be found the in example directory, or in the package that specifically relates to their usage, such as the perception packages.

Over time, the repo will grow to include more types of manipulators.

## Contributing
Feel free to send PRs to add features to currently existing Arm ROS packages or to include new ones. Note that all PRs should follow the structure and naming conventions outlined in the repo including documentation.

## Contributors
- [Solomon Wiznitzer](https://github.com/swiz23) - **ROS Engineer**
- [Luke Schmitt](https://github.com/lsinterbotix) - **Robotics Software Engineer**
- [Levi Todes](https://github.com/LeTo37) - **CAD Engineer**

## Citing

If using this software for your research, please include the following citation in your publications:

```bibtex
@software{Wiznitzer_interbotix_ros_manipulators,
  author = {Wiznitzer, Solomon and Schmitt, Luke and Trossen, Matt},
  license = {BSD-3-Clause},
  title = {{interbotix_ros_manipulators}},
  url = {https://github.com/Interbotix/interbotix_ros_manipulators}
}
