#returnofthecups

###INSTALLATION

1. opencv
2. libfreeenct
3. libftdi

<p>Preferably use a package manager (use homebrew on OSX) - this ensures that dependencies
are correctly installed.</p>

<p>Note to self: need to use libftdi1 in makefile</p>

###FILES

#####Vision
* device.cpp / device.h : the abstracted freenect device class
* cups.cpp / cups.h : functions to help identify and located cups
* main.cpp : the main program

####Robot
<p>This required a rebuild in the Multi_Dynamixel_Robot_Control folder - need to make a better Makefile without the QT rubbish</p>