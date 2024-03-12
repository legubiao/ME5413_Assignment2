# ME5413 AMR Assignment

compile this package
```
catkin_make -DCATKIN_WHITELIST_PACKAGES="me5413"
```

## Assignment 2

### Task 1 Cartographer

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
get lidar rotation and transformation in imu link
```
rosrun tf tf_echo imu_link velo_link
```

launch fast-lio and play rosbag
```
roslaunch me5413 task2_fastlio.launch
rosbag play --clock YOUR_ROSBAG_FILE
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


### Task 2 Coordinate transformation code
#### from_odometry_output_kitti.py:

Change "/aft_mapped_to_init_high_frec" to the topic that needs to be subscribed to odometry.

Change "camera_gray_left" to the coordinate system node which groundtruth is located.

Change "velo_link" to the coordinate system node which the odometry result is located.

Run the py program in the bag and play another bag containing odometry and tf topics. The code will convert the odometry results into the coordinate system where the groundtruth node is located, and output the txt file of kitti format coordinates in the same directory.

#### from_odometry_output_tum.py:

Change "/aft_mapped_to_init" to the topic that needs to be subscribed to odometry.

Change "camera_gray_left" to the coordinate system node which groundtruth is located.

Change "camera_init" to the coordinate system node which the odometry result is located.

Run the py program in the bag and play another bag containing odometry and tf topics. The code will convert the odometry results into the coordinate system where the groundtruth node is located, and output the txt file of tum format coordinates in the same directory.


### Task 2 Eveluation recorded rosbag
run python script read data from rosbag directly
```
roscd me5413

python scripts/read_tum_from_rosbag.py '/home/biao/catkin_ws/src/ME5413_AMR/output/task2/fast_lio.bag' --topic /Odometry

evo_ape tum have_fun_tum.txt odom_tum.txt -r full --plot --plot_mode xz
```
or, run the node and play rosbag
```
rosrun me5413 record_tum.py
rosbag play YOUR_ROSBAG_FILE
```
