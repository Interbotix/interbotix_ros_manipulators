# interbotix_xsarm_gravity_compensation

## Overview
This package demos the interbotix_gravity_compensation package on an Interbotix arm. As of now, the supported arms include: WidowX-250 6DOF and ALOHA WidowX-250 6DOF.

## Usage
Run the following launch command where `robot_model` is a mandatory choice between `aloha_wx250s` and `wx250s`, `robot_name` defaults to be the same as `robot_model` but can be anything, and `motor_specs` defaults to `<path_to_this_package>/config/motor_specs_<robot_model>.yaml`:
```
ros2 launch interbotix_xsarm_gravity_compensation interbotix_gravity_compensation.launch.py robot_model:=xxx [robot_name:=xxx] [motor_specs:=xxx]
```
It runs the `interbotix_gravity_compensation` node and launches the xsarm_control script to bring up the arm.

Then, enable/disable the gravity compensation with the following service call:
```
ros2 service call /wx250s/gravity_compensation_enable std_srvs/srv/SetBool 'data: [true/false]'
```

The arm will hold itself against gravity and can be moved freely when the gravity compensation is enabled. It will lock in its current position when the gravity compensation is disabled.

**WARNING: the arm WILL torque off and drop for a short period of time while enabling/disabling. Please make sure it is in a resting position or manually held.**

**WARNING: the joints not supporting current control WILL torque off. Please make sure to use arm with at least the first three joints supporting current control, e.g., RX, WX, VX series.**

## Configuration
The `motor_specs.yaml` hosts the motor specifications used in the node and provides knobs for motor assistance against joint frictions. A template file is given below:
```
# Motor Assist: scale the no-load currents which alleviate the effects of friction
# If the values are invalid, they defaults to 0
motor_assist:
  # Set 'all' to [0, 1] to scale the no load currents of all joints uniformly
  # Or to -1 and use joint specific values
  all: -1
  # Set the joint specific values to [0, 1] to scale differently for each joint
  waist: 0.5
  shoulder: 0.5
  elbow: 0.5
  forearm_roll: 0.5
  wrist_angle: 0.5
  wrist_rotate: 0.5

motor_specs:
  waist:
    # torque constant (Nm/A): how much torque is produced per Amp of current
    torque_constant: 1.793
    # current unit (A): how much current command is needed to produce 1 Amp of current
    current_unit: 0.00269
    # no load current (A): the maximum no load current applied when motor_assist == 1
    # It should be as large as possible without the joint accelerating by itself
    no_load_current: 0.1

  shoulder:
    torque_constant: 1.793
    current_unit: 0.00269
    no_load_current: 0.0

  elbow:
    torque_constant: 1.793
    current_unit: 0.00269
    no_load_current: 0.0

  forearm_roll:
    torque_constant: 0.897
    current_unit: 0.00269
    no_load_current: 0.1

  wrist_angle:
    torque_constant: 0.897
    current_unit: 0.00269
    no_load_current: 0.0

  wrist_rotate:
    torque_constant: 0.897
    current_unit: 0.00269
    no_load_current: 0.1

# Joints specified here but not in motor_assist or motor_specs
# do not support the current control mode
joint_names:
  [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper]

```

## Running on Other Arms
As of now, only the WidowX-250 6DOF and ALOHA WidowX-250 6DOF arms are tested and verified to work with the gravity compensation package. However, this package should be compatible with all RX, WX, VX series arms. To run the demo on those arms, one need to put together the corresponding `motor_specs_xxx.yaml`. The torque constants and current units can be looked up from the ROBOTIS e-manual ([example](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)) according to the arm specification ([example](https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/awx250s.html)). One may also need to adjust the URDF inertia entries in the [interbotix_xsarm_description](https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf) package to provide the node with an accurate mass distribution information.