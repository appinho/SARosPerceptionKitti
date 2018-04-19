# ROS package for Perception of the KITTI dataset

<p align="center">
  <img src="./videos/semantic.gif">
</p>

<p align="center">
  <img src="./videos/rviz.gif">
</p>

## Evaluation Result for Scenario 0060

| Class        | CAR           | PEDESTRIAN  |
| ------------ |:-------------:|:-----------:|
| MOTA         | 0.832168      | 0.390625    |
| MOTP         | 0.703046      | 0.689011    |
| MOTAL        | 0.832168      | 0.390625    |
| MODA         | 0.832168      | 0.390625    |
| MODP         | 0.714124      | 0.858284    |

## Pipeline

* [Semantic segmentation](https://github.com/martinkersner/train-DeepLab)
* [Kitti Dataset to rosbag](https://github.com/tomas789/kitti2bag)
