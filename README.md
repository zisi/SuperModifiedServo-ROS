## Super Modified Servo - ROS
This repository contains C library that used from ROS in order to control [Super Modified Servo](http://www.01mechatronics.com/product/supermodified-v30-rc-servos).
It is inspired by [this arduino library](http://www.01mechatronics.com/support/downloads).
The description for all functions you can find [here](http://www.01mechatronics.com/sites/default/files/docs/SuperModified.V3.Datasheet.pdf).
This ROS package tested in fedora 22 with ROS indigo - 1.11.14.
## Installation
To install the ROS package, clone the repository files in your ROS workspace:
```bash
roscd
cd ../src
git clone https://github.com/zisi/SuperModifiedServo-ROS.git
cd ..
catkin_make #Build the package
```
## Testing
Connect the controller to USB-RS485 or USB-UART as describe [here](http://www.01mechatronics.com/support/gettingstarted/test485).
To find the device name:
```bash
$ dmesg | grep tty
```
Then change the device name in src/Commander.cpp

    int fd = serialPortOpen("device name", B"baud rate");

for example serialPortOpen("/dev/ttyUSB0", B115200).

For testing run the launch file:
```bash
$ roslaunch super_modified_servo SuperModifiedTest.launch
```
In other terminal run the "rqt_reconfigure" node:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
<img src="https://raw.github.com/zisi/SuperModifiedServo-ROS/master/pics/rqt_reconfigure.png" width="80%" height="80%" />

With rqt_reconfigure, the user can control the velocity of the super_modified_servo
and change some parameters. The implemented functions are:
* search: find the ID of connected super_modified_servo
* start: start the super_modified_servo with ID "Motor ID"
* stop: stop the super_modified_servo with ID "Motor ID"
* reset: run the "resetIncrementalPosition()" function, with ID "Motor ID"
* setID: change the "Motor ID" of the super_modified_servo to "New motor ID"
* setBaud: change the baud rate to "Set baud rate"
* error_reaction: set and get error reaction, the "errorReaction[20]" is default.
* setGain: set the gains of PID position controller to P gain, I gain, D gain values.
* getGain: get the gains of PID position controller.

The user can type the above commands in the "Send command" field.
Also information or error messages appear in the "roslaunch" terminal.
In the "Velocity set point" field, the user can change the velocity of super_modified_servo.
In the "P gain", "I gain", "D gain", the user can change the control gains of super_modified_servo.

The state of motor is published in /JointsState topic:
```bash
rostopic echo /JointsState
```
or you can plot it by using:
```bash
rosrun rqt_plot rqt_plot
```
<img src="https://raw.github.com/zisi/SuperModifiedServo-ROS/master/pics/rqt_plot.png" width="80%" height="80%" />

## License
#LGPLv2.1

## Known problems
* If the user in your platform does not have permissions to read/write to serial device, run the roslaunch as super user or give him the permissions for this.
* Communication with more than one super_modified_servo in high rate communication causes problem.
