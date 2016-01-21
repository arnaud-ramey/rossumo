# rossumo

![jumping_sumo](https://raw.githubusercontent.com/arnaud-ramey/rossumo/master/doc/jumping_sumo.jpg)

Description
===========

"rossumo" is a driver to use the Jumping Sumo robot, built by Parrot, in ROS.
It relies on [ARDroneSDK3](http://developer.parrot.com/docs/bebop/?c#general-information),
the official Parrot DSK.

Supported hardware
==================

The library was developed for the original Parrot Jumping Sumo.
However, it should be compatible with the newer versions.

Licence
=======

See LICENCE

Usage
=====

See `launch/rossumo.launch` for an example of usage.

Install
=======

You first need to install the official SDK (ARDrone3) by Parrot.

Dependencies
------------
``` c++
$ sudo apt-get install phablet-tools autoconf
```

Download ARDroneSDK3
--------------------
Following [the instructions](http://developer.parrot.com/docs/bebop/?c#download-all-sources):
``` c++
$ repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git
$ repo sync
```

Build ARDroneSDK3
-----------------
``` c++
$ ./build.sh -p Unix-forall -t build-sdk -j
```

Build ARDroneSDK3 samples
-------------------------
``` c++
$ git clone https://github.com/Parrot-Developers/Samples.git
$ cd Samples/Unix/JumpingSumoPiloting
```
  Change the lines in the Makefile:
    SDK_DIR=/home/arnaud/sumo/out/Unix-base/staging/usr
    CFLAGS=-I$(IDIR) -I $(SDK_DIR)/include/
    LDIR = $(SDK_DIR)/lib/
    <check the different -L flags>
    <add json to libs>
``` c++
$ LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/arnaud/sumo/out/Unix-base/staging/usr/lib ./JumpingSumoPiloting
$ sudo sh -c 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/arnaud/sumo/out/Unix-base/staging/usr/lib ./JumpingSumoPiloting '
```

Build rossumo
-------------
``` c++
$ catkin_make --only-pkg-with-deps rossumo
```
