# ME5524 Bayesian Robotics - Fall 2018

Workspace for the MBZIRC 2020 Challenge 3. The objective of this package is to locate a fire using the Clearpath Jackal UGV and navigate towards it.

## Table of Contents
1. [Installation](#1-installation)
2. [Operation](#2-operation)

## 1. Installation
### Install prerequisites
```bash
sudo apt-get update
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install protobuf-compiler python-pip python-toml python-jinja2 python-catkin-tools
sudo pip install --upgrade pip
sudo pip install -U catkin_tool
```

### Install this workspace
```bash
cd ~
git clone https://github.com/Abdullah-Abduldayem/ws_drone_legs.git
cd ~/ws_drone_legs
```

### Configure Environment

Add this to your `~/.bashrc` file.

```bash
nano ~/.bashrc

## Add the following, save and quit
source $HOME/ws_drone_legs/devel/setup.bash
```

### Download Workspace Dependencies
```bash
cd ~/ws_drone_legs
wstool update -t src
```

### Build Workspace
```
cd ~/ws_drone_legs
catkin build
```


## 2. Operation

### Configure ROS MASTER
The following must be run on each terminal to communicate properly with the jackal. To avoid doing this every time, you can include these in you `~/.bashrc` file.
```
export ROS_MASTER_URI=http://cpr-jackal:11311
export ROS_IP=<your computer's IP address>
```

### Start the webcam and lidar
```
roslaunch mbz_c3_jackal gscam.launch
rosrun hokuyo_node hokuyo_node
```

### Run the ball detection
```
rosrun mbz_c3_jackal ball_detection.py 
```

### Getting the Jackal to work with keyboard teleop
After installing the `teleop_twist_keyboard` package, fire up the node and control Jackal using the keyboard

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```