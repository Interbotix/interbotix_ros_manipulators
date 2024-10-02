# interbotix_xsarm_gravity_compensation

## Overview

This package demos the interbotix_gravity_compensation package on an Interbotix arm.
As of now, the supported arms include: WidowX-250 6DOF and ALOHA WidowX-250 6DOF.

## Configuration

Please refer to [`this section`](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/gravity_compensation.html#configuration) for details.

## Usage

Run the following launch command where `robot_model` is a mandatory choice between `aloha_wx250s` and `wx250s`, `robot_name` defaults to be the same as `robot_model` but can be anything, and `motor_specs` defaults to `<path_to_this_package>/config/motor_specs_<robot_model>.yaml`:
```
ros2 launch interbotix_xsarm_gravity_compensation interbotix_gravity_compensation.launch.py robot_model:=xxx [robot_name:=xxx] [motor_specs:=xxx]
```
It runs the `gravity_compensation` node and launches the xsarm_control script to bring up the arm.

Then, enable/disable the gravity compensation with the following service call:
```
ros2 service call /<robot_name>/gravity_compensation_enable std_srvs/srv/SetBool 'data: [true/false]'
```

The arm will hold itself against gravity and can be moved freely when the gravity compensation is enabled.
It will lock in its current position when the gravity compensation is disabled.


> [!WARNING]
> WARNING: the arm WILL torque off and drop for a short period of time while enabling/disabling. Please make sure it is in a resting position or manually held.

> [!WARNING]
> WARNING: the joints not supporting current control WILL torque off. Please make sure to use arm with at least the first three joints supporting current control, e.g., RX, WX, VX series.
