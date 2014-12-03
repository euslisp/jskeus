## [JSK EusLisp repository](http://jskeus.sourceforge.net) [![Build Status](https://travis-ci.org/euslisp/jskeus.png?branch=master)](https://travis-ci.org/euslisp/jskeus)


This repository containts software devleoped and used by [JSK](http://www.jsk.t.u-tokyo.ac.jp )at The University of Tokyo.

## 1. Getting started

### 1.1 Install binary (Ubuntu 12.04)

#### 1.1.1 Setup ROS repository
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
sudo apt-get update
```
#### 1.1.2 Install Euslisp
```
sudo apt-get install ros-hydro-euslisp
```

### 1.2 Install from source code
#### 1.2.1 Installing dependent libraries

for Ubuntu users
```
$ sudo apt-get install git gcc g++ libjpeg-dev libxext-dev libx11-dev libgl1-mesa-dev libglu1-mesa-dev libpq-dev libpng12-dev xfonts-100dpi xfonts-75dpi msttcorefonts
```

for Ubuntu 11.04 or later, install more fonts.
```
$ sudo apt-get install gsfonts-x11 texlive-fonts-extra xfonts-100dpi-transcoded xfonts-75dpi-transcoded
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
$ sudo apt-get install git gcc g++ libjpeg62-dev libxext-dev libx11-dev libgl1-mesa-dev libglu1-mesa-dev libpq-dev libpng12-dev xfonts-100dpi xfonts-75dpi
## additional install for using OpenGL on raspberry pi
$ sudo apt-get install libgl1-mesa-dri xserver-xorg-video-vesa xutils
```
for Cygwin(now working with 32bit only)
```
Packages listed below should be intalled using setup.exe.

git, gcc-core, gcc-g++, make
libjpeg-devel, libpng-devel
libGL-devel, libGLU-devel, libglut-devel, libX11-devel, libXext-devel
xorg-server
X-start-menu-icons (for startxwin.exe)
mesa-demo (for glxinfo)
bitmap fonts (all bitmap font packages -75dpi, -100dpi, -misc )
```


#### 1.2.2 Downloading and building EusLisp
```
$ git clone http://github.com/euslisp/jskeus jskeus
$ cd jskeus
$ make
you will see instructions to add EUSDIR, ARCHDIR, PATH, LD_LIBRARY_PATH environment variable in your bashrc to invoke euslisp program
$ echo "source ~/bashrc.eus" >> ~/.bashrc
```

## 2 Playing with Demo programs

### 2.1 Motion generation
```
$ irteusgl irteus/demo/demo.l
```
and type any command, such as '(crank-motion)' or '(dual-arm-ik)'


![full-body-ik](images/Full-body-ik.png)
![dual-arm-ik](images/Dual-arm-ik.png)
![dual-manip-ik](images/Dual-manip-ik.png)
![crank-motion](images/Crank-motion.png)
![hand-grasp](images/Hand-grasp.png)
![hanoi-arm](images/Hanoi-arm.png)
![particle](images/Particle.png)

### 2.2 Robots and objects models
```
$ irteusgl models/irt-all-robots.l "(make-all-robots)"
```
![all robots](images/irt-all-robots.png)

```
$ irteusgl models/irt-all-objects.l "(make-all-objects)"
```
![all objects](images/irt-all-objects.png)

## 3 Getting started for ROS and PR2 users

### 3.1 Install binaries (Ubuntu 12.04)

#### 3.1.1 Setup ROS repository

You can skip this procdeure if you already setup ROS system
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
sudo apt-get update
```
#### 3.1.2 Install pr2/euslisp programs

```
sudo apt-get install ros-hydro-pr2eus
```

### 3.2 Install from source

#### 3.2.1  Install ROS and jsk-ros-pkg repository

See http://wiki.ros.org/hydro/Installation/Source for more detail

```
$ sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
$ mkdir -p ~/ros_catkin_ws/src
$ cd ~/ros_catkin_ws/src
$ wstool init src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_pr2eus/master/.rosinstall
$ wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_travis/master/rosdep-install.sh
$ sh ./rosdep-install.sh
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

### 3.3 Running demo programs

#### 3.3.1 Visualizinot pr2 robot
```
$ rosrun euscollada pr2.sh
```

![./pr2.sh](images/Pr2eus.png)

#### 3.3.2 For real PR2 users

After you set `ROS_MASTER_URI` and `ROS_IP` to your PR2 robot (or simulated PR2), try following command

```
$ roscd pr2eus
;; read joint state and display in euslisp viewer, print jacobian
$ roslaunch ./pr2-read-state.launch
;; sample program to show how to make pr2 posture from euslisp, THIS WILL MOVE REAL ROBOT, CAUTION!!!
$ roslaunch ./pr2-send-joints.launch
```

You can use our script to set  `ROS_MASTER_URI` and `ROS_IP`
```
$ source `rospack find jsk_tools`/src/bashrc.ros
$ rossetrobot pr1012.jsk.com # change to your environment
$ rossetip

```

![./pr2-read-state.launch](images/Pr2-read-state.png)


see roseus package in http://github.com/jsk-ros-pkg repository for ROS client library for euslisp

### 3.4 Support and Trouble Shooting


Use [issue tracker](https://github.com/euslisp/jskeus/issues) for get support

Use [pull requests](https://github.com/euslisp/jskeus/pulls) to report bugs or patches.

### 3.5 Online Documents (Currently only Japanese is avilable)

See online [manual](http://euslisp.github.io/jskeus/)

PDF files are also available from [here](https://github.com/euslisp/jskeus/raw/master/doc/jmanual.pdf)

### 3.6 Acknowledgment

The software in this repository is based on [EusLisp language](http://euslisp.sourceforge.net).
