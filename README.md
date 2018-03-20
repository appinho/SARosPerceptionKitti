# Perception/Multi-Object Tracking of the KITTI Dataset in ROS Framework

[//]: # (Image References)

Change pic
[image1]: ./images/rqt_plot.png "rqt_plot visualization"
## Video of RVIZ

Change video
The resulting video of RViz can be found on YouTube by clicking on the image below:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/pQ3EL-UoUpI/0.jpg)](https://www.youtube.com/watch?v=pQ3EL-UoUpI)

### 0 To Dos  
- Change DA to be GNN  
- Include ground extraction and interpolation of point cloud
- Add DBSCAN cluster algorithm  (Only seed at dynamical voxels) 
- Add orientation, height and z coordinate in detection  
- Include Geometrical and Semantical Model/Classification of tracks not only by semantic but also by dimension  
- Plot RMSE   
- Plot text to bounding boxes   
- Plot ego car    
- Plot detection, tracking, ground truth position on diagram and see smoothing effect of tracking filter  
- Create better DA step   
- Apply methods to all videos   
Optional  
- Include Stereo Camera  
- Improve Semantic Segmentation by Transfer Learning or better Network architecture  
- Exchange UKF with PF

### 1 Installations

#### a) Ubuntu 16   
https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-ubuntu#0   
https://youtu.be/5NtCb7rdipo?t=10m4s

#### b) ROS Kinetic
http://wiki.ros.org/kinetic/Installation/Ubuntu   

### c) Coding & Editors
Sublime
http://tipsonubuntu.com/2017/05/30/install-sublime-text-3-ubuntu-16-04-official-way/

### d) Drivers
Ubuntu Terminal: sudo apt-get install nvidia-384

### e) Git
Connect the ROS node with GitHub the first time   
https://help.github.com/articles/adding-an-existing-project-to-github-using-the-command-line/   
Set git config name and email

### 2 Dataset
http://www.cvlibs.net/datasets/kitti/raw_data.php   
https://github.com/tomas789/kitti2bag
Explain semantic images and time synchronisation

### 3 ROS

Roslaunch  
Explain ros messages packages nodes and helper classes

### 4 RQT_PLOT

![alt text][image1]   

To show element of an array: https://answers.ros.org/question/249736/plot-posestamped-topic-in-rqt_plotrviz/
Show trajectories  


After converting the Kitti Data to a rosbag file like in topic 2 and using tutorial   
https://github.com/udacity/didi-competition/blob/master/docs/GettingStarted.md
#### 1 Pausing rosbag replay with space bar
#### 2 Change fixed frame to velodyne
#### 3 Add 'by topic' PointCloud2
