# ME5413 AMR Assignment



## Assignment 2

### task 1

launch command 

```
roslaunch me5413 ME5413.launch bag_filename:=${HOME}/2dlidar.bag
```

### Task 2
clone fast-lio to src folder and compile
```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
git clone https://github.com/Livox-SDK/livox_ros_driver.git

cd FAST_LIO
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
```

launch fast-lio
```
roslaunch me5413 task2_fastlio.launch
```
play kitti rosbag
```
rosbag play ~/have_fun.bag
```

get lidar rotation and transformation in imu link
```
rosrun tf tf_echo imu_link velo_link
```
record rosbag
```
rosbag record /Odometry /tf /tf_static
```