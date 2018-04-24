#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <geometry_msgs/Point.h>
//#include <perception_ros_kitti/parameter.h>
//#include <perception_ros_kitti_msgs/Object.h>

using namespace Eigen;
using namespace geometry_msgs;
//using namespace perception_ros_kitti_msgs;

class Tools{

public:

	Tools();
	~Tools();

	MatrixXf getImage2DBoundingBox(
		const Point & point,
		const float width,
		const float height);
	//MatrixXf getImage2DBoundingBox(
	//	const Object o);

	// Transformation functions
	MatrixXf transformVeloToCam(const MatrixXf & velo_points);
	MatrixXf transformCamToRectCam(const MatrixXf & cam_points);
	MatrixXf transformRectCamToImage(const MatrixXf & rect_cam_points);
	MatrixXf transformVeloToImage(const MatrixXf & velo_points);

	int getClusterKernel(const int semantic);

	// Semantic helpers
	std::vector<std::string> SEMANTIC_NAMES;
	std::map<int, int> SEMANTIC_COLOR_TO_CLASS;
	MatrixXi SEMANTIC_CLASS_TO_COLOR;
	VectorXi SEMANTIC_KERNEL_SIZE;
	
private:

	// Transformation
	MatrixXf TRANS_VELO_TO_CAM;
	MatrixXf TRANS_CAM_TO_RECTCAM;
	MatrixXf TRANS_RECTCAM_TO_IMAGE;

};