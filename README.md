# BOSCH SR450 Robot at CTU/CIIRC

Package to control CTU/CIIRC robot Bosch SR450 via MARS control unit.
This code is based on `https://github.com/cvut/pyrocon` but adds packaging,
documentation, and some testing.
For the complete documentation see https://robotics-robot-stations.readthedocs.io/

## Installation

```bash
pip install ctu_bosch_sr450
```

## Usage

```python
from ctu_bosch_sr450 import RobotBosch

robot = RobotBosch()  # set argument tty_dev=None if you are not connected to robot, it will allow you to compute FK and IK offline
robot.initialize()  # initialize connection to the robot
robot.move_to_q([0.1, 0.0, 0.0, 0.0])  # move robot
robot.wait_for_motion_stop()
robot.close()  # close the connection
```

### Kinematics

```python
robot = RobotBosch(tty_dev=None)  # initialize object without connection to the robot
x, y, z, phi = robot.fk([0, 0, 0, 0])  # compute forward kinematics
q = robot.ik([x, y, z, phi])[0]  # compute inverse kinematics, get the first solution
```

## Coordinate systems

The library uses meters and radians for all values.
Variable __q__ is used to denote joint configuration, i.e. the array of joint
angles/joint distance for revolute and prismatic joints, respectively.
Variables __x__, __y__, __z__, and __phi__ are used to denote position and orientation
of the end-effector in the base frame. The orientation is given as rotation around the
z-axis of the base frame.
The reference base frame is located as shown in the figure below.

![](https://raw.githubusercontent.com/CTURobotics/ctu_bosch_sr450/main/doc/base_frame.png)

## Joint configuration

The joint configuration is defined as follows:

- the first joint is revolute and its angle is measured w.r.t. the x-axis of the base
  frame
- the second joint is revolute and is measured w.r.t. the previous link
- the third joint is prismatic and controls the height (i.e. motion in z-axis)
- the last joint is revolute and measured w.r.t. the x-axis of the base frame (i.e. *
  *not** w.r.t. the previous link)

## How to control the robot

__In case robot does anything unexpected, press the emergency button immediately.__

### Starting the robot

- power up the robot with red switch on the Mars control panel
- create RobotBosh() instance and call initialize()
- you will be asked to press the yellow button (Arm Power) on the Mars control panel
- robot will perform homing after which you are able to control it with this library

### Finishing the work with the robot

- to turn robot of call soft_home() followed by the close() methods
- turn red switch off

### Entering the cage

- to enter the cage, you need to call release() function, that will power-off the motors
  and activate breaks

### Recovering from error

#### Emergency stop or cage entry

- unblock emergency stop button and/or closed the cage
- press red button called MotionStop on the Mars control panel
- press yellow button called ArmPower on the Mars control panel
- continue with normal operation

#### Motor error

- in case green led above motor axis is blinking and red led is on, there is a motor
  error
- to reset motors call reset_motors() method

## Hard home

Hard home is needed after the control unit power was turned off. It is performed
automatically in the initialize() method. However, it needs to be performed only after
the power was turned off and on again. In case the connection needs to be reestablished,
you can call initialize without homing by setting the argument home to False.

## Acknowledgment

Preparation of the course materials was partially supported by:

![](https://raw.githubusercontent.com/CTURobotics/ctu_bosch_sr450/main/doc/NPO-publicity-logo_crop.png)



## For developers
```
git clone git@github.com:CTURobotics/ctu_bosch_sr450.git
cd ctu_bosch_sr450
uv sync
uv pip install -e ".[dev]"
pre-commit install
```
