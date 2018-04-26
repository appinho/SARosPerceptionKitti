# ROS package for Perception of the KITTI dataset

## Result

<p align="center">
  <img src="./videos/semantic.gif">
</p>

<p align="center">
  <img src="./videos/rviz.gif">
</p>

## Evaluation for 7 Scenarios 0011,0013,0014,0018,0056,0059,0060

| Class        | CAR           | PEDESTRIAN  |
| ------------ |:-------------:|:-----------:|
| MOTA         | 0.148286      | -0.191729   |
| MOTP         | 0.693174      |  0.592259   |
| MOTAL        | 0.168644      | -0.191729   |
| MODA         | 0.832168      | -0.191729   |
| MODP         | 0.168984      |  0.962181   |

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

