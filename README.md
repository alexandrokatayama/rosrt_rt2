# Package Summary

rosrt_rt2 is a packege of ROS node to manipulate RT.works' robot assist walker RT.2.
This explains how to use it.

**NOTE:RT.works does not support using RT.2 with ROS. This may break the product and it is not covered by warranty.**

# Preparation

## Hardware

Open the control box and find connector J1102.
PIN #1 and PIN #2 are for serial communication(pulled up to 3.3V), and PIN #3 is GND.

Get USB to TTL Serial Converter Cable
(For example : http://akizukidenshi.com/catalog/g/gM-05840 )
and connect those 3 pins to the cable.

**NOTE:communication lines are pulled up to 3.3v. Choose proper converter cable.**

Pin asignments are:

PIN #1 of J1102(RXD pulled up to 3.3v) --- PIN #4(ORANGE) of Converter

PIN #2 of J1102(TXD pulled up to 3.3v) --- PIN #5(YELLOW) of Converter

PIN #3 of J1102(GND) --- PIN #1(BLACK) of Converder

## Software

get source codes from Github repository(https://github.com/alexandrokatayama/rosrt_rt2.git )

File	|Explanation
--	|--
src/rosrt_rt2.cpp | source code of ROS node.
msg/Rt2Sensor.msg | definition of message from the node.
script/sample.py  | sample script to send command to the node.

Declare the src/rosrt_rt2.cpp as executable in CMakeLists.txt.
Declare the msg/Rt2Sensor.msg as message in CMakeLists.txt.
Then build them in the workspace.

# Operation

## Power up the RT.2

Push power button on the control box.

## On the PC

### Start roscore
	$ roscore
### Set up the serial port
```
# stty -F /dev/ttyUSB0 raw -echo speed 115200
```
If you need to use another port like /dev/ttyUSB1, edit the source code of the node.
### Start the node
```
$ rosrun ros_start rosrt_rt2
```
### Start monitoring sensor values
```
$ rostopic echo /rosrt_rt2
```
### Start script
```
$ rosrun ros_start sample.py /mobile_base/commands/velocity:=/cmd_vel
```
Then RT.2 moves forward, backward, and turn left and right.
