# ROS package for Perception of the KITTI dataset

## Result

<p align="center">
  <img src="./videos/semantic.gif">
</p>

<p align="center">
  <img src="./videos/rviz.gif">
</p>

## Evaluation for Scenario 0060

| Class        | CAR           | PEDESTRIAN  |
| ------------ |:-------------:|:-----------:|
| MOTA         | 0.832168      | 0.390625    |
| MOTP         | 0.703046      | 0.689011    |
| MOTAL        | 0.832168      | 0.390625    |
| MODA         | 0.832168      | 0.390625    |
| MODP         | 0.714124      | 0.858284    |

## Pipeline

### 0 Data acquisition

* [Kitti Dataset to rosbag](https://github.com/tomas789/kitti2bag)

### 1a) Sensor Fusion: Velodyne Point Cloud Processing

### 1b) Sensor Fusion: Raw Image Processing

* [Semantic segmentation](https://github.com/martinkersner/train-DeepLab)

### 1c) Sensor Fusion: Mapping Point Cloud and Image

### 2 Detection: DBSCAN Clustering

### 3 Tracking: UKF Tracker

