# interbotix_xsarm_joy

## Overview
This package can be used to control the movements of any X-Series robotic arm using a SONY PS3/PS4 controller or Microsoft Xbox360 controller (untested) via Bluetooth. In this demo, the 'arm' joints are set to work in 'position' control mode while the gripper operates in 'PWM' mode. Refer to the joystick button map below to see how to operate the robot. Specifically, some of the joystick controls manipulate individual joints while others are used to perform 'inverse kinematics' on all the joints to get the end-effector of the robot (defined at 'ee_gripper_link') to move as if it's in Cartesian space. This is done using the [modern_robotics](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python) code library offered by Northwestern University.

## Structure
![xsarm_joy_flowchart](images/xsarm_joy_flowchart.png)
As shown above, the *interbotix_xsarm_joy* package builds on top of the *interbotix_xsarm_control* package. To get pointers about the nodes in that package, please look at its README. The other nodes are described below:
- **joy** - a ROS driver for a generic Linux joystick; it reads data from a joystick over Bluetooth and publishes  [sensor_msgs/Joy](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Joy.html) messages to the `commands/joy_raw` topic
- **xsarm_joy** - responsible for reading in raw [sensor_msgs/Joy](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Joy.html) messages from the `commands/joy_raw` topic and converting them into [ArmJoy](msg/ArmJoy.msg) messages; this makes the code more readable and allows users to remap buttons very easily later.
- **xsarm_robot** - responsible for reading in [ArmJoy](msg/ArmJoy.msg) messages and sending joint and gripper commands to the **xs_sdk** node; while the 'waist' joint can be directly controlled via the PS3/PS4 joystick, other buttons allow position-ik to be performed using all the arm joints.

## Bluetooth Setup
#### Sony PS4 Controller (Recommended)
Getting a PS4 controller connected via Bluetooth to a Linux laptop is pretty straightforward. Click the *Bluetooth* icon on the top right of your screen, followed by *Bluetooth Settings...*. Next, press and hold the *Share* button on the PS4 controller (see image below for reference). While holding the *Share* button, press and hold the *PS* button. After a few seconds, the triangular shaped LED located between the *L2* and *R2* buttons should start rapidly flashing white (about twice a second) at which point you can let go.

On the computer, click the '+' icon in the *Bluetooth* settings window. Wait until you see 'Wireless Controller' pop up, select it, and click *Next* on the bottom right of the window. A message should display saying 'successfully set up new device 'Wireless Controller'' and the LED should turn blue. This means the controller is connected to the computer. To disconnect, hold down the *PS* button for about 10 seconds until the LED turns off. To reconnect, just press the *PS* button (no need to hold it down). After blinking white a few times, the LED should turn blue.

![ps4](images/ps4.jpg)

##### Sony PS3 Controller
Getting a PS3 controller connected via Bluetooth to a Linux laptop can be a bit finicky at times. However, the commands below should do the trick. Get an original SONY PS3 controller, it's accompanying USB cable, and open up a terminal. Type:
```
$ sudo bluetoothctl
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# scan on
```
Now, plug the PS3 controller into the Linux Laptop. At this point, a message should pop up in the terminal that looks something like the following (with a different MAC address):
```
[NEW] Device FC:62:B9:3F:79:E7 PLAYSTATION(R)3 Controller
```
When it appears, type:
```
[bluetooth]# trust <MAC-address>
```
Now unplug the PS3 controller and press the PS button. The four red LEDs at the front of the controller should flash a few times, eventually leaving just one LED on by the '1'. This means that the joystick paired successfully.

Sometimes, the joystick might cause the cursor of the computer mouse to go crazy. To fix this, add the following line to the `~/.bashrc` file:
```
alias joy_stop='xinput set-prop "PLAYSTATION(R)3 Controller" "Device Enabled" 0'
```
Now, whenever the PS3 joystick is paired to the computer, just type `joy_stop` in the terminal to stop it messing with the mouse (you're welcome).

## Usage
After pairing the joystick, type the following in a terminal (let's say to control the WidowX 200 arm):
```
roslaunch interbotix_xsarm_joy xsarm_joy.launch robot_model:=wx200
```
A red error message might appear in the screen saying `Couldn't open joystick force feedback!`. This is normal and will not affect the joystick operation. To further customize the launch file at run-time, look at the table below:

| Argument | Description | Default Value |
| -------- | ----------- | :-----------: |
| robot_model | model type of the Interbotix Arm such as 'wx200' or 'rx150' | "" |
| robot_name | name of the robot (typically equal to `robot_model`, but could be anything) | "$(arg robot_model)" |
| base_link_frame | name of the 'root' link on the arm; typically 'base_link', but can be changed if attaching the arm to a mobile base that already has a 'base_link' frame| 'base_link' |
| use_rviz | launches Rviz | true |
| mode_configs | the file path to the 'mode config' YAML file | refer to [xsarm_control.launch](launch/xsarm_control.launch) |
| threshold | value from 0 to 1 defining joystick sensitivity; a larger number means the joystick should be less sensitive | 0.75 |
| controller | type of controller ('ps3', 'ps4', or 'xbox360') | ps4 |
| launch_driver | true if the *xsarm_control.launch* file should be launched - set to false if you would like to run your own version of this file separately | true |
| use_sim | if true, the Dynamixel simulator node is run; use Rviz to visualize the robot's motion; if false, the real Dynamixel driver node is run | false |

To understand how the joystick buttons map to controlling the robot, look at the diagram and table below. Note that while the Xbox360 has a slightly different naming convention, the button placement is essentially the same:

![ps3](images/ps3.jpg)

| Button | Action |
| ------ | ------ |
| START/OPTIONS | move robot arm to its Home pose |
| SELECT/SHARE | move robot arm to its Sleep pose |
| R2 | rotate the 'waist' joint clockwise |
| L2 | rotate the 'waist' joint counterclockwise |
| Triangle | increase gripper pressure in 0.125 step increments (max is 1)|
| X | decrease gripper pressure in 0.125 step increments (min is 0)|
| O | open gripper |
| Square | close gripper |
| D-pad Up | increase the control loop rate in 1 Hz step increments (max of 40) |
| D-pad Down | decrease the control loop rate in 1 Hz step increments (min of 10) |
| D-pad Left | 'coarse' control - sets the control loop rate to a user-preset 'fast' rate |
| D-pad Right | 'fine' control - sets the control loop rate to a user-preset 'slow' rate |
| Right stick Up/Down | increase/decrease pitch of the end-effector |
| Right stick Left/Right | increase/decrease roll of the end-effector |
| R3 | reverses the Right stick Left/Right control |
| Left stick Up/Down | move the end-effector (defined at 'ee_gripper_link') vertically in Cartesian space |
| Left stick Left/Right | move the end-effector (defined at 'ee_gripper_link') horizontally in Cartesian space |
| L3 | reverses the Left stick Left/Right control |
| R1 | if the arm has 6dof, this moves the end-effector in a negative direction along its own 'y' axis |
| L1 | if the arm has 6dof, this moves the end-effector in a positive direction along its own 'y' axis |
| PS | if torqued on, holding for 3 seconds will torque off the robot; if torqued off, tapping the button will torque on the robot |
