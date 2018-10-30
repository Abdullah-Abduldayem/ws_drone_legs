# Drone Legs (ROS)
Quadrapedal legs added to a UAV in order to improve landing stability,
allow for aerial maipulation, and add the possibility of ground-based
locomotion or perching behaviors.

## Installation
```
cd <catkin_ws>/src
git clone https://github.com/Abdullah-Abduldayem/drone_legs_ros.git
cd ..
catkin_make
```


## Running
To run the program, run the following:

```
roslaunch drone_legs_ros gazebo_setup.launch
```


## Troubleshooting
### Gazebo Plugin

If you are getting the following error message:
```
[Err] [Plugin.hh:165] Failed to load plugin libgazebo_drone_legs_control_plugin.so: libgazebo_drone_legs_control_plugin.so: cannot open shared object file: No such file or directory
```
then add the catkin devel folder to the GAZEBO_PLUGIN_PATH:

```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib
```

To avoid having to run this command every time before `roslaunch`, add the command to 
the bottom of your `~/.bashrc` file. This will make sure the GAZEBO_PLUGIN_PATH is
properly set every time you open a new terminal window.



## Feature List
The following is a list of features to be included in future releases:

* Increase friction in simulation
* Simulate soft toes?
* Deal with extension during momentary lost of contact with ground

* Create more varied environments (sloped terrain, bumpy terrain, seesaw/controlled slope)
* Correct orientation imbalance
* Adapt to varying fall speeds