## Install ROS on RPi Stretch

Follow the instructions here for install ROS on the RaspberryPi. It is recommended to do this in a new directory to avoid mixing `catkin build` with `catkin_make`

```
mkdir ~/catkin_ws_build
cd ~/catkin_ws_build
```

[http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)

Increase the swapfile on the Raspberry Pi, otherwise you will get an internal compiler error:

```
sudo nano /etc/dphys-swapfile

## Make the following change and save it:
## CONF_SWAPSIZE=2048
```

Once that's done, reboot and install some dependencies:

```
sudo apt-get install libgeographic-dev libpoco-dev libeigen3-dev libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev

cd ~/catkin_ws_build
rosinstall_generator angles control_toolbox diagnostic_msgs diagnostic_updater eigen_conversions geographic_msgs geometry_msgs nav_msgs pluginlib rosconsole_bridge sensor_msgs tf tf2_ros tf2_eigen urdf visualization_msgs --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall
wstool merge -t src kinetic-custom_ros.rosinstall
#wstool update -t src
wstool update -t src -j4
#wstool update -t src -j2
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic
```

## Install MAVROS

Follow the instructions under "Source Installation"

[https://dev.px4.io/en/ros/mavros_installation.html#source-installation](https://dev.px4.io/en/ros/mavros_installation.html#source-installation)