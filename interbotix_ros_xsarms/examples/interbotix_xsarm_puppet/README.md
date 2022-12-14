# interbotix_xsarm_puppet

## Overview

This package demos two similar features that the Interbotix arms support: Record & Playback, and Arm Puppeteering.

### Record & Playback

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/record_and_playback.html)

Imagine that you would like to have the robot arm perform some motions to achieve a specific task. One way of doing this would be to create a [JointTrajectory](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html) of desired joint positions at specific times which you could then command the robot. Alternatively (and much less time consuming), you could manually manipulate the arm to do the specific motions and record them in a ROS bag file. Then, you could 'play back' the bag file as many times as you like to repeat the motions on the same robot later. This 'record/playback' feature is made possible by the **xsarm_puppet_single**.

### Arm Puppeteering

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/arm_puppeteering.html)

Now imagine that you have two (or more) Interbotix arms with the same number of joints. What the **xsarm_puppet** allows you to do is to manually manipulate one of the arms and watch as the motion is repeated in real time on the second robot. A potential application of this could be a warehousing environment. Instead of a worker straining his back to lift a heavy package, he could manually manipulate a small version of a robotic arm to maneuver an industrial-sized arm to pick it up.
