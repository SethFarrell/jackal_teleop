# jackal_teleop
Joystick teleop modified from https://github.com/ros-teleop/teleop_twist_joy for personal use

Added feature to adjust linear/angular speeds through button pressed.

| Button | Function                |
| ------ | ------------------------|
| Y      | Increase linear speed   |
| B      | Decrease linear speed   |
| X      | Increase angular speed  |
| A      | Decrease angular speed  |

# Running in Gazebo
1. Install clearpath's packages:

http://wiki.ros.org/ClearpathRobotics/Packages

`sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation`

2. Clone this repo into a workspace, build, source, and launch:

`roslaunch jackal_teleop start-with-gazebo.launch`

This assumes you have a joystick in /dev/input/js0

You can modify teleop.launch to point to your correct joystick if necessary:

`<arg name="joy_dev" default="/dev/input/js0" />`


# Running on an actual jackel
1. Set up a workspace and clone this repo along with jackals packages into /src

https://github.com/jackal/jackal_robot

https://github.com/jackal/jackal

build and source your workspace, start your robot by launching 'base.launch' from the jackal_robot package (make sure jackal_port is set up correctly to point to your jackal robot)

`roslaunch jackal_robot base.launch`

Either modify the line in base.launch that starts the default jackal teleop node:

`<include file="$(find jackal_control)/launch/teleop.launch" />`

to point to our package

`<include file="$(find jackal_teleop)/launch/teleop.launch" />`

or after running base.launch start our teleop node which uses the same name and will shut theirs down before taking over.

`roslaunch jackal_teleop start-real-jackal.launch`







# Common Issues
1. Verify the joystick is on and the node is receiving input. You can check this by using 'rostopic echo /bluetooth_teleop/joy' and pressing button the joystick to see if the output changes.

2. If using a real robot verify the base.launch is using the correct port to communication with the jackal. By default it looks for JACKAL_PORT which you can check on the robot by running 'ls -a /dev' and seeing if it is there. For us it was called ttyACM0
