#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  // input->data.size()
  for(int i = 0; i < 10; ++i){
    geometry_msgs::Point p;
    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;
    memcpy(&X, &input->data[16*i], sizeof(float));
    memcpy(&Y, &input->data[16*i] + 4, sizeof(float));
    memcpy(&Z, &input->data[16*i] + 8, sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;
    //ROS_INFO("Entry [%g]",X);
    ROS_INFO("Coordinates X,Y,Z: [%g],[%g],[%g]", p.x, p.y, p.z);
    //output.data.push_back(input->data[i]);
  }
  std::cout << std::endl;
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "kitti_pcl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kitti/velo/pointcloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}