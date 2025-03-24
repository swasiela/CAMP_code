# INSTALL

## Requirements

- Ubuntu-20.04. 

## Getting started

This document will take you step-by-step through the installation of the various dependencies required to properly install the project from scratch.

- Make sure **you have cloned the project before** proceeding, as it contains the Python requirements file. 

- Make sure your system is **up-to-date** by executing the following commands in a terminal:
```
sudo apt update && sudo apt upgrade
```

## ROS 

First of all, you need to install ROS, since the project uses ROS messages to communicate between different applications.

> ROS noetic installation wiki: **[ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)**

### Setup

Open a new terminal and run:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Make sure you have curl by running:

```
sudo apt install curl
```

Then setup your ROS repository key by running:

```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

### Installation

In the same terminal execute:

```
sudo apt install ros-noetic-desktop-full
```

Then install ros dependencies by running:

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### Environment

You must source this script in every bash terminal you use ROS in by executing:

```
source /opt/ros/noetic/setup.bash
```

>We recommend adding this directly to your bashrc so that ROS is automatically sourced in each new terminal by running :
>```
>echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
>```

## Bullet3

> **WARNING** If you're using a virtual machine, please choose one allowing GPU pass-through.

### Installation

Then, you need to install **[bullet3](https://github.com/bulletphysics/bullet3)** from the sources in order to get all the libraries. You can install it where you want on your computer but avoid installing it in your project workspace.

```
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
mkdir build && mkdir install
export BULLET_PATH=$(pwd)
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$BULLET_PATH/install -DBUILD_SHARED_LIBS=ON
make install
```

> We recommend modifing the b3Printf and b3Warning functions located in Bullet3Common/b3Logging.h in order to prevent your feed to get spamed. 
> Modify the following functions at the top of the file: #define b3Printf(...) do {;}while (0) and #define b3Warning(...) do {;}while (0)

### Environment

Bullet is now installed. In order for the library to be recognized when the project is compiled, you must export the following variables to your bashrc:

```
echo "export BULLET_INSTALL_PATH=**your_bullet3_path**/install" >> ~/.bashrc && source ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$BULLET_INSTALL_PATH/lib" >> ~/.bashrc && source ~/.bashrc
```

## Python

This project uses Python3.

### Setup

Make sure you have already cloned the CAMP repository.

### Installation

Start by installing pip3: 
```
sudo apt install python3-pip
```

Then head into the CAMP repository and install the requirements:
```
cd **your_CAMP_path** && pip3 install -r python_requirements.txt
```

This project is build using the *catkin build* command which is part of the *catkin_tools*. To install it run: 
```
sudo apt-get install python3-catkin-tools
```

Robot models in python are compiled with JITCODE using *clang*, which you need to install by running :
```
sudo apt install clang
```

## (**Optional**) Blender
> This step is **optional** and is only useful if you wish to modify the environments provided or a 3D visualization of the results. 

Download **[blender](https://www.blender.org/download/)** and unzip it in the location of your choice.

Don't follow the installation instruction. To run it, open a terminal, *cd* to your blender folder and then run:
```
./blender
```

## (**Optional**) Genom3 and Matlab

> This step is **optional** and requires a **Matlab license**. 
> Genom3 and matlab are used only for Gazebo simulations.
> Note that the main simulator used to generate the results of the various references is based on the python robot models and bullet3 located in the planning package. 


```
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:**your_CAMP_path**/src/simulation_genom/genom_simu/models" >> ~/.bashrc && source ~/.bashrc
```

