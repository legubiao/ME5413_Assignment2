# ME5413 AMR Assignment

compile this package
```
catkin_make -DCATKIN_WHITELIST_PACKAGES="me5413"
```

## Assignment 2

### task 1

launch command 

```
roslaunch me5413 task1.launch bag_filename:=${HOME}/下载/2dlidar.bag
```

### Task 2 Fast-LIO
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

launch fast-lio and play rosbag
```
roslaunch me5413 task2_fastlio.launch bag_filename:=${HOME}/下载/have_fun.bag
```

get lidar rotation and transformation in imu link
```
rosrun tf tf_echo imu_link velo_link
```
record rosbag
```
rosbag record /Odometry /tf /tf_static
```

### Task 2 VINS-Fusion

clone vins-fusion to src folder and compile

```
cd ~/catkin_ws/src
git clone https://github.com/guisoares9/VINS-Fusion

cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="vins"
```
launch rviz and loop fusion node
```
roslaunch me5413 task2_vinsfusion.launch
```
rosrun vins node
```
rosrun vins vins_node /home/biao/catkin_ws/src/ME5413_AMR/config/vins-fusion/kitti_bag.yaml
```
record rosbag
```
rosbag record /vins_estimator/odometry /vins_estimator/point_cloud /vins_estimator/camera_pose /tf /tf_static
```

### eveluation bag
```
roscd me5413

python scripts/read_tum_from_rosbag.py '/home/biao/catkin_ws/src/ME5413_AMR/output/task2/fast_lio.bag' --topic /Odometry

evo_ape tum have_fun_tum.txt odom_tum.txt -r full --plot --plot_mode xz
```