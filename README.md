#returnofthecups

###INSTALLATION

1. Install OpenCV
2. Install libfreeenct
3. Install Eigen
4. Install FTDI drivers http://www.ftdichip.com/Drivers/VCP.htm

If Eigen libraries are not found, run the following commands:

* cd /usr/local/include
* sudo ln -sf eigen3/Eigen Eigen
* sudo ln -sf eigen3/unsupported unsupport

Preferably use a package manager (use homebrew on OSX) - this ensures that dependencies
are correctly installed.

###FILES

* device.cpp / device.h : the abstracted freenect device class
* cups.cpp / cups.h : functions to help identify and located cups
* main.cpp : the main progra