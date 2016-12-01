# pyDevantechRE #

Python classes to interface to [Devantech's / robot-electronics's](http://www.robot-electronics.co.uk/) products.

## General remarks ##

This development came up as a quick&dirty implementation to get some devices up'n'running on a BeagleBone Black platform. Nevertheless, some care has been taken to try to be as platform independent as possible and to implement most of the devices' functionality.

OS development/testing platforms include:

* Angstrom
* Ubuntu 14.04

## Development model ##

This project is following [Vincent Driessn's git branching model](http://nvie.com/posts/a-successful-git-branching-model/) (or, at least, trying). A good introduction can be found in [Jeff Kreeftmeijer's post](http://jeffkreeftmeijer.com/2010/why-arent-you-using-git-flow/). Therefore, any contribution is recommended to follow that model.

### Docstring 'format' ###

Just to make things consistent, Numpy style docstrings are being used in the project. It is appreciated that contributions also follow those guidelines.

## Software dependencies ##

* [pySerial](http://pyserial.sourceforge.net/) for serial I/O
* [Adafruit_BBIO](https://learn.adafruit.com/setting-up-io-python-library-on-beaglebone-black/overview) for BeagleBone specific I/O

## List of implemented devices ##

* [USBI2C](http://www.robot-electronics.co.uk/htm/usb_i2c_tech.htm): USB to I2C interface module.
* [SD21](http://www.robot-electronics.co.uk/htm/sd21tech.htm): Servo controller.
* [MD25](http://www.robot-electronics.co.uk/htm/md25tech.htm): Dual motor controller.

## (extremely short) Version history ##

### 0.1.2 ###
* Corrected README file.

### 0.1.1 ###
* Full (well, almost) implementation for all 3 devices. The missing part being the serial protocol implementation for the MD25.
* Basic inline code documentation done.
* Broken python packaging (just started a skeleton of setup.py).
* Not thoroughly tested.

### 0.1.0 ###
* First release.
* Implementation incomplete.
* Basic inline code documentation missing.
