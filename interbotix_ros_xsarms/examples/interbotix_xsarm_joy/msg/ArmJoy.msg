# Send commands from the xsarm_joy node to the xsarm_robot node

# enum values that define the joystick controls for the robot

#########################################################################################################

# Control the motion of the virtual 'ee_gripper_link' or end effector using the modern_robotics_ik engine
# Position Control
int8 EE_X_INC = 1
int8 EE_X_DEC = 2
int8 EE_Y_INC = 3
int8 EE_Y_DEC = 4
int8 EE_Z_INC = 5
int8 EE_Z_DEC = 6

# Orientation Control
int8 EE_ROLL_CCW = 7
int8 EE_ROLL_CW = 8
int8 EE_PITCH_UP = 9
int8 EE_PITCH_DOWN = 10

#########################################################################################################

# Control the motion of independent joints on the Arm or send predefined robot poses
# Waist Joint Control
int8 WAIST_CCW = 11
int8 WAIST_CW = 12

# Gripper Control
int8 GRIPPER_OPEN = 13
int8 GRIPPER_CLOSE = 14

# Pose Control
int8 HOME_POSE = 15
int8 SLEEP_POSE = 16

#########################################################################################################

# Customize configurations for the Interbotix Arm
# Inc/Dec Joint speed
int8 SPEED_INC = 17
int8 SPEED_DEC = 18

# Quickly toggle between a fast and slow speed setting
int8 SPEED_COURSE = 19
int8 SPEED_FINE = 20

# Inc/Dec Gripper pressure
int8 GRIPPER_PWM_INC = 21
int8 GRIPPER_PWM_DEC = 22

# Torque On/Off all servos
int8 TORQUE_ON = 23
int8 TORQUE_OFF = 24

#########################################################################################################

# Control the motion of the Interbotix Arm
int8 ee_x_cmd
int8 ee_y_cmd
int8 ee_z_cmd
int8 ee_roll_cmd
int8 ee_pitch_cmd

# Independent Joint/Pose Control
int8 waist_cmd
int8 gripper_cmd
int8 pose_cmd

# Misc. Configs
int8 speed_cmd
int8 speed_toggle_cmd
int8 gripper_pwm_cmd
int8 torque_cmd
