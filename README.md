# rossumo

<p align="center">
<img src="doc/jumping_sumo.jpg"
   alt="sumo" style="width: 300px"/>
<img src="doc/ros_indigoigloo_600.png"
   alt="MiP" style="width: 200px"/>
</p>

Description
===========

"rossumo" is a driver to use the Jumping Sumo robot, built by Parrot, in ROS.
It relies on
[ARDroneSDK3](http://developer.parrot.com/docs/bebop/?c#general-information),
the official Parrot DSK.
It is written in C++.


Supported hardware
==================

The library was developed for the original Parrot Jumping Sumo,
as shown in the picture.
However, it should work seamlessly with the newer versions
(Jumping Race Drones and Jumping Night Drones).

**Supported firmware**: v1.99.0.
The list and changelog of firmwares is available
[here](http://www.parrot.com/usa/support/parrot-jumping-sumo/).

Licence
=======

LGPL v3 (GNU Lesser General Public License version 3).
See LICENCE.

ROS driver node
===============

To launch the Sumo driver:

```bash
$ roslaunch rossumo rossumo.launch
```

Node parameters
---------------

- `camera_calibration_filename`
[std::string, default: ""]

If not empty, the path to the calibration file of the camera.
For instance, `$(find rossumo)/data/sumo_camera_parameters.yaml`

- `camera_calibration_camname`
[std::string, default: "camname"]

Name of the camera in the calibration file of the camera.
For instance, `$(find rossumo)/data/sumo_camera_parameters.yaml`

Subscriptions
-------------

- `cmd_vel`
[geometry_msgs::Twist, (m/s, rad/s)]

The instantaneous speed order.
Send it every 10 Hz to obtain continuous motion.

- `anim`
[std_msgs::String]

Play one of the predefined animations,
among `metronome`, `ondulation`, `slalom` `slowshake`, `spin`,
      `spinJump`, `spinToPosture`, `spiral`, `tap`.

- `set_posture`
[std_msgs::String]

Play one of the predefined postures,
among `standing`, `kicker`, `jumper`.

- `sharp_turn`
[std_msgs::Float32, radians]

Make a on-the-spot turn.
Positive angles generate CCW turns.

- `high_jump`
[std_msgs::Empty]

Perform a high jump (about 80 cm high).

- `long_jump`
[std_msgs::Empty]

Perform a long jump (about 80 cm long).

Publications
------------

- `camera/image_raw`
[sensor_msgs::Image]

The 640x480 raw image, encoded as `bgr8`.
The framerate is roughly 15 fps.
The image comes from the MJPEG video stream of the robot.
If there is no subscriber to the topic,
the streaming is stopped from the robot,
which saves battery.

- `camera/camera_info`
[sensor_msgs::CameraInfo]

The camera_info read from a calibration file.

- `battery_percentage`
[std_msgs::Int16, 0~100]

The percentage of remaining battery.

- `posture`
[std_msgs::String]

The current predefined posture
among `unknown`, `standing`, `kicker`, `jumper`.

- `link_quality`
[std_msgs::Int16, 0~5]

Quality of the Wifi connection,
between 0 (very bad) and 5 (very good).

- `alert`
[std_msgs::String]

The alerts emitted by the robot.
Current they only concern the battery level,
among `unkwnown`, `none`, `low_battery`, `critical_battery`

- `outdoor`
[std_msgs::Int16]

TODO

Keyboard remote control
=======================

To launch remote control of the Sumo thanks to keyboard:

```bash
$ roslaunch rossumo joy_teleop.launch
```

It is based on the [`turtlebot_teleop`](https://github.com/turtlebot/turtlebot/tree/indigo/turtlebot_teleop) package.

Joystick remote control
=======================

To launch remote control of the Sumo thanks to joystick:

```bash
$ roslaunch rossumo joy_teleop.launch
```

It is based on the [`joy`](http://wiki.ros.org/joy) package.

Installation
============

You first need to install the official SDK (ARDrone3) by Parrot.
A summary of the instructions comes below.


Dependencies
------------

```bash
$ sudo apt-get install phablet-tools autoconf
```

**FFMPEG** for Trusty: you need the latest version of ```ffmpeg```.
Use the [official PPA](https://launchpad.net/~mc3man/+archive/ubuntu/trusty-media):

```bash
$ sudo add-apt-repository ppa:mc3man/trusty-media
$ sudo apt-get update
$ sudo apt-get dist-upgrade
```

Download ARDroneSDK3
--------------------

Following [the instructions](http://developer.parrot.com/docs/bebop/?c#download-all-sources):

```bash
$ repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git
$ repo sync
```


Build ARDroneSDK3
-----------------

```bash
$ ./build.sh -p Unix-forall -t build-sdk -j
```


Build ARDroneSDK3 samples
-------------------------

```bash
$ git clone https://github.com/Parrot-Developers/Samples.git
$ cd Samples/Unix/JumpingSumoPiloting
```

Change the lines in the Makefile:

```makefile
SDK_DIR=/home/arnaud/sumo/out/Unix-base/staging/usr
CFLAGS=-I$(IDIR) -I $(SDK_DIR)/include/
LDIR = $(SDK_DIR)/lib/
<check the different -L flags>
<add json to libs>
```

```bash
$ LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/arnaud/sumo/out/Unix-base/staging/usr/lib ./JumpingSumoPiloting
$ sudo sh -c 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/arnaud/sumo/out/Unix-base/staging/usr/lib ./JumpingSumoPiloting '
```


Build rossumo
-------------

```bash
$ catkin_make --only-pkg-with-deps rossumo
```


Camera calibration
==================

Following the instructions of camera_calibration
[wiki page](http://wiki.ros.org/camera_calibration) and
[tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) :

```bash
$ rosrun camera_calibration cameracalibrator.py --size 8x10 --square 0.0298 image:=/rossumo1/rgb camera:=/camera
```

