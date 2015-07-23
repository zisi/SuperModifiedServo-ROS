# Super Modified Servo - ROS
This repository contains C library that used from ROS in order to control [Super Modified Servo](http://www.01mechatronics.com/product/supermodified-v30-rc-servos). It is inspired by [this arduino library](http://www.01mechatronics.com/support/downloads). This ROS package tested in Xubuntu 14.04 with ROS indigo - 1.11.13
# Instalation
To install the ROS package, clone the repository files in your ROS workspace:
```bash
roscd
cd ../src
git clone https://github.com/zisi/SuperModifiedServo-ROS.git
cd ..
catkin_make #Compile the files
```
# Testing
Connect the controller to USB-RS485 or USB-UART as describe [here](http://www.01mechatronics.com/support/gettingstarted/test485).
To find the device name:
```bash
$ dmesg | grep tty
```
Then change the device name in src/SuperModifiedTest.cpp

    int fd = serialPortOpen("[device name]");

for example "/dev/ttyUSB0" .

For testing run the launch file:
```bash
$ roslaunch super_modified_servo SuperModifiedTest.launch
```
and in other terminal write the command to the topic /Command:
```bash
# test 1
$ rostopic pub -1 /Command super_modified_servo/Command "cmd: 'b1'"
# or for test 2
$ rostopic pub -1 /Command super_modified_servo/Command "cmd: 'b2'"
# or for test 3
$ rostopic pub -1 /Command super_modified_servo/Command "cmd: 'b3'"  
```
# Known problems
* If the user in your platform does not have permissions to read/write to serial device, run the roslaunch as super user or give him the permissions for this.