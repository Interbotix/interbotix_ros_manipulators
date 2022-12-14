# interbotix_xsarm_descriptions

[![View Documentation](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_descriptions.html)

## Overview

This package contains the URDFs and meshes for the many X-Series Interbotix Arms. The STL files for each robot are located in a unique folder inside the [meshes](meshes/) directory. Also in the 'meshes' directory is the [interbotix_black.png](meshes/interbotix_black.png) picture. The appearance and texture of the robots come from this picture. Next, the URDFs for the robot are located in the [urdf](urdf/) directory. They are written in 'xacro' format so that users have the ability to customize what parts of the URDF get loaded to the parameter server. Note that all the other Interbotix X-Series Arms core ROS packages reference this package to launch the robot description.
