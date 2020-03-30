# OpenZen Node for ROS

This software allows to forward sensor data from sensor connected via OpenZen to ROS.

To use it in your ROS setup, follow these steps:
```
mkdir -p catskin_ws/src
cd catskin_ws/src

git clone --recurse-submodules https://bitbucket.org/lpresearch/openzenros.git

# get your ROS environment going
source /opt/ros/melodic/setup.bash
catkin_make
source ./devel/setup.bash
```

Open another terminal window and run the ROS core:

```
roscore
```

To allow access to sensors connected via USB, you need to ensure that the user running the ROS sensor node
has access to the /dev/ttyUSB devices. You can do this by adding the user to the dialout group.

```
sudo adduser <username> dialout
```

After this call, you should logout and login with this user to ensure the changed permissions are in effect.

You can then run the OpenZen ROS driver with this command:

```
rosrun openzen_sensor openzen_sensor
```

By default, it will connect to the first available sensor. If you want to connect to
a specific sensor, you can use the serial name of the sensor as parameter, for example:

```
rosrun openzen_sensor openzen_sensor _sensor_name:="LPMSCU2000573"
```

Now you can print the IMU values from ROS with:

```
rostopic echo /imu
```

Or plot some values (for example linear acceleration) with 

```
rosrun rqt_plot rqt_plot "/imu/linear_acceleration/"
```

If you want to readout the values of two OpenZen sensors simultanously, you need to rename the topics and the node names likes this:

```
rosrun openzen_sensor openzen_sensor __name:="cu2node" _sensor_name:="LPMSCU2000573" imu:=/cu2_imu mag:=/cu2_mag/
rosrun openzen_sensor openzen_sensor __name:="ig1_node" _sensor_name:="LPMSIG1000032" imu:=/ig1_imu mag:=/ig1_mag/
```
