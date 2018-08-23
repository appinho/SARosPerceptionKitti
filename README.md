# ROS package for Perception of the KITTI dataset

<p align="center">
  <img src="./videos/semantic.gif">
</p>

<p align="center">
  <img src="./videos/rviz.gif">
</p>

## How to run the package

1) [Install ROS Kinetic on Ubuntu 16.04] (http://wiki.ros.org/kinetic/Installation/Ubuntu)
2) [Create ROS Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) by opening a new Terminal and run following commands:  
```
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws/src  
git pull https://github.com/appinho/ROS_Perception_Package_Kitti_Dataset.git  
cd ..  
catkin_make  
source devel/setup.bash  
```
3) Converting a scenario (e.g. 0060 like the video above) from the [KITTI Dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to a ROSbag file with [Kitti2Bag](https://github.com/tomas789/kitti2bag)

* Download synced+rectified scenario and its calibration file
* Store the two files in a separate directory (e.g. /data/)
* Unzip them

## Evaluation for 7 Scenarios 0011,0013,0014,0018,0056,0059,0060

| Class        | MOTA    | MOTP    |  MOTAL  |    MODA |    MODP |
| ------------ |:-------:|:-------:|:-------:|:-------:|:-------:|
| CAR          | 0.250970| 0.715273| 0.274552| 0.274903| 0.785403|
| PEDESTRIAN   |-0.015038| 0.581809|-0.015038|-0.015038| 0.988038|


[157, 154, 280, 306, 378, 1283, 17]
[64, 10, 10, 72, 11, 196, 0]
[39, 75, 120, 39, 33, 569, 0]

[8, 0, 1, 0, 4, 18, 18]
[3, 0, 2, 0, 0, 52, 0]
[172, 0, 63, 0, 25, 177, 46]

## Pipeline

### 1a) Sensor Fusion: Velodyne Point Cloud Processing

* [Ground extraction & Free space estimation](http://wiki.ros.org/but_velodyne_proc)

### 1b) Sensor Fusion: Raw Image Processing

* [Semantic segmentation](https://github.com/martinkersner/train-DeepLab)

### 1c) Sensor Fusion: Mapping Point Cloud and Image

### 2 Detection: DBSCAN Clustering

### 3 Tracking: UKF Tracker

