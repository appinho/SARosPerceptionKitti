# kitti_ros

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

### 3 Visualization   
Terminal 1: roscore   
Terminal 2: rviz    
Terminal 3: rosbag play kitti_2011_09_26_drive_0005_synced.bag    
Terminal 4: rosrun test_kitti kitti_subscriber    

After converting the Kitti Data to a rosbag file like in topic 2 and using tutorial   
https://github.com/udacity/didi-competition/blob/master/docs/GettingStarted.md
#### 1 Pausing rosbag replay with space bar
#### 2 Change fixed frame to velodyne
#### 3 Add 'by topic' PointCloud2
