# rossumo

![jumping_sumo](https://raw.githubusercontent.com/arnaud-ramey/rossumo/master/doc/jumping_sumo.jpg)

Description
===========

"rossumo" is a driver to use the Jumping Sumo robot, built by Parrot, in ROS.

Supported hardware
==================

The library was developed for the original Parrot Jumping Sumo.

Licence
=======

See LICENCE

Usage
=====

See `launch/rossumo.launch` for an example of usage.

Install
=====

You first need to install the official SDK (ARDrone3) by Parrot.

Dependencies
------------
  $ sudo apt-get install phablet-tools autoconf

Download, following [the instructions](http://developer.parrot.com/docs/bebop/?c#download-all-sources):
  $ repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git
  $ repo sync

Build
-----
  $ ./build.sh -p Unix-forall -t build-sdk -j

Build samples
-------------
  $ git clone https://github.com/Parrot-Developers/Samples.git
  $ cd Samples/Unix/JumpingSumoPiloting
  Change the lines in the Makefile:
    SDK_DIR=/home/arnaud/sumo/out/Unix-base/staging/usr
    CFLAGS=-I$(IDIR) -I $(SDK_DIR)/include/
    LDIR = $(SDK_DIR)/lib/
    <check the different -L flags>
    <add json to libs>
  $ LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/arnaud/sumo/out/Unix-base/staging/usr/lib ./JumpingSumoPiloting
  $ sudo sh -c 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/arnaud/sumo/out/Unix-base/staging/usr/lib ./JumpingSumoPiloting '
