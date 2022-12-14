# interbotix_xsarm_pid

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/pid_gains_test_environment.html)

## Overview

This package can be used as a way to test 'pwm' or 'current' PID gains when operating the arm in either 'pwm' or 'current' mode. PID gains are read into the controller node from the [gains.yaml](config/gains.yaml) file. The node then commands the arm to its 'home' pose and waits ten seconds for it to settle. Then it commands the arm to its 'sleep' pose and waits another ten seconds for it to settle. Finally, the node commands all the motors to either zero pwm or zero current (effectively torquing them off) before shutting itself down.
