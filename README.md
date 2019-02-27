# Drone Legs

ROS workspace for a quadrupedal drone meant for handling difficult terrain.

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
git clone https://github.com/adayem/ws_drone_legs.git
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

### Install pyssc32
```
cd ~/ws_drone_legs/src/pyssc32
sudo python setup.py install
```

### Instal DynamixelSDK
```bash
cd ~/ws_drone_legs/src/DynamixelSDK/python/
sudo python setup.py install
```


## 2. Operation

### Run leg controller (Real-world)
```
rosrun drone_legs_ros leg_state_publisher.py
```
