## [JSK EusLisp repository](http://jskeus.sourceforge.net)


This repository containts software devleoped and used by [JSK](http://www.jsk.t.u-tokyo.ac.jp )at The University of Tokyo.

### Getting started

#### Installing dependent libraries

for Ubuntu users
```
$ sudo apt-get install subversion gcc g++ libjpeg-dev libxext-dev libx11-dev libgl1-mesa-dev libglu1-mesa-dev libpq-dev libpng12-dev xfonts-100dpi xfonts-75dpi msttcorefonts
```
for Mac OSX users using MacPorts
```
$ sudo port install jpeg libpng mesa xorg-libX11 wget
```
for Mac OSX users using Homebrew
```
$ brew install jpeg libpng mesalib-glw wget
```

'''NOTE:'''
EusLisp needs X11 Library, but Mac OSX mountain lion no longer support it. So you need to install substitution [XQuartz](http://xquartz.macosforge.org/landing/).

Also for Mac OSX lion (or older) users it's better to install the latest XQuartz for the safety.

(If XQuartz is older than 2.7.3, the installation of EusLisp will fail. You should install the latest XQuartz.)


for Raspberry Pi users using debian(Raspbian "wheezy")
```
$ sudo apt-get install subversion gcc g++ libjpeg62-dev libxext-dev libx11-dev libgl1-mesa-dev libglu1-mesa-dev libpq-dev libpng12-dev xfonts-100dpi xfonts-75dpi
## additional install for using OpenGL on raspberry pi
$ sudo apt-get libgl1-mesa-dri xserver-xorg-video-vesa xutil
```
for Cygwin(now working with 32bit only)
```
Packages listed below should be intalled using setup.exe.

subversion, gcc-core, gcc-g++, make
libjpeg-devel, libpng-devel
libGL-devel, libGLU-devel, libglut-devel, libX11-devel, libXext-devel
xorg-server
X-start-menu-icons (for startxwin.exe)
mesa-demo (for glxinfo)
bitmap fonts (all bitmap font packages -75dpi, -100dpi, -misc )
```


#### Downloading and building EusLisp
```
$ git clone http://github.com/euslisp/jskeus jskeus
$ cd jskeus
$ make
you will see instructions to add EUSDIR, ARCHDIR, PATH, LD_LIBRARY_PATH environment variable in your bashrc to invoke euslisp program
$ echo "source ~/bashrc.eus" >> ~/.bashrc
```

#### Demo program
```
$ irteusgl irteus/demo/demo.l
```

![full-body-ik](images/Full-body-ik.png)
![dual-arm-ik](images/Dual-arm-ik.png)
![dual-manip-ik](images/Dual-manip-ik.png)
![crank-motion](images/Crank-motion.png)
![hand-grasp](images/Hand-grasp.png)
![hanoi-arm](images/Hanoi-arm.png)
![particle](images/Particle.png)

#### Robots and objects models
```
$ irteusgl models/irt-all-robots.l "(make-all-robots)"
```
![all robots](images/irt-all-robots.png)

```
$ irteusgl models/irt-all-objects.l "(make-all-objects)"
```
![all objects](images/irt-all-objects.png)

### Getting started for ROS and PR2 users

* Install ROS and jsk-ros-pkg repository (See http://www.ros.org/wiki/ROS/Installation/rosinstall for more detail)
```
$ sudo apt-get install python-setuptools
$ sudo easy_install -U rosinstall
$ rosinstall ~/ros/cturtle http://www.ros.org/rosinstalls/cturtle_pr2.rosinstall
$ rosinstall ~/ros/cturtle http://jsk-ros-pkg.svn.sourceforge.net/viewvc/jsk-ros-pkg/trunk/jsk.rosinstall
$ echo "source ~/ros/cturtle/setup.sh" >> ~/.bashrc
$ . ~/.bashrc
```

* for ROS users

euslisp compile
```
$ rosdep install euslisp
$ rosmake euslisp
```
demo program
```
$ rosdep install euscollada
$ roscd euscollada
$ rosmake
$ ./pr2.sh
```

![./pr2.sh](images/Pr2eus.png)

* for real PR2 users
```
$ roscd pr2eus
$ rosmake
;; read joint state and display in euslisp viewer, print jacobian
$ roslaunch ./pr2-read-state.launch
;; sample program to show how to make pr2 posture from euslisp
$ roslaunch ./pr2-send-joints.launch
```

![./pr2-read-state.launch](images/Pr2-read-state.png)


see roseus package in http://github.com/jsk-ros-pkg repository for ROS client library for euslisp

### Support and Trouble Shooting


Use [issue tracker](https://github.com/euslisp/jskeus/issues) for get support

Use [pull requests](https://github.com/euslisp/jskeus/pulls) to report bugs or patches.

### Online Documents


See online [manual](http://jskeus.sourceforge.net/html/manual.html). (Japanese translation [manual](http://jskeus.sourceforge.net/html/jmanual.html manual))

PDF files are also available from [here](https://svn.code.sf.net/p/euslisp/code/trunk/EusLisp/doc/latex/manual.pdf) (Japanese translation from [here](https://svn.code.sf.net/p/euslisp/code/trunk/EusLisp/doc/jlatex/jmanual.pdf))

### Acknowledgment

The software in this repository is based on [EusLisp language](http://euslisp.sourceforge.net).
