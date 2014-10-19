#returnofthecups

###INSTALLATION

1. opencv
2. libfreeenct

<p>Preferably use a package manager (use homebrew on OSX) - this ensures that dependencies
are correctly installed.</p>

<p>Note to self: need to use libftdi1 in makefile</p>

###FILES

#####Vision
* device.cpp / device.h : the abstracted freenect device class
* cups.cpp / cups.h : functions to help identify and located cups
* main.cpp : the main program

####Robot
<p> On OSX the serial device will appear as <b>/dev/cu.xxxxx</b> and <b>/dev/tty.xxxxx</b>. <b>cu</b> is the "callout" device, it's what you use when you establish a connection to the serial device and start talking to it. `tty` is the "dialin" device, used for monitoring a port for incoming calls for e.g. a fax listener. To that end, the connection must be established to <b>cu</b>.</p>
