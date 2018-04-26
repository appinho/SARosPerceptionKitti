# ROS package for Perception of the KITTI dataset

## Result

<p align="center">
  <img src="./videos/semantic.gif">
</p>

<p align="center">
  <img src="./videos/rviz.gif">
</p>

## Evaluation for 7 Scenarios 0011,0013,0014,0018,0056,0059,0060

| Class        | MOTA    | MOTP    |  MOTAL  |    MODA |    MODP |
| ------------ |:-------:|:-------:|:-------:|:-------:|:-------:|
| CAR          | 0.148286| 0.693174| 0.168644| 0.832168| 0.168984|
| PEDESTRIAN   |-0.191729| 0.592259|-0.191729|-0.191729| 0.962181|


[175, 185, 221, 66, 188, 910, 0]
[722, 175, 258, 113, 417, 1542, 110]
[69, 0, 15, 0, 0, 110, 4]
[76, 0, 64, 0, 29, 170, 33]

## Pipeline

### 0 Data acquisition

* [Kitti Dataset to rosbag](https://github.com/tomas789/kitti2bag)

### 1a) Sensor Fusion: Velodyne Point Cloud Processing

* [Ground extraction & Free space estimation](http://wiki.ros.org/but_velodyne_proc)

### 1b) Sensor Fusion: Raw Image Processing

* [Semantic segmentation](https://github.com/martinkersner/train-DeepLab)

### 1c) Sensor Fusion: Mapping Point Cloud and Image

### 2 Detection: DBSCAN Clustering

### 3 Tracking: UKF Tracker

