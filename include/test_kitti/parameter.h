/*
 *	Header for project parameters
 */

// TRA Transformation and coordinate system parameters
const float TRA_X_OFFSET = 0; //-0.6; //-0.81;

// PCL Point cloud parameters
const bool PCL_FILTER_POINTCLOUD = true;
const float PCL_FILTER_MIN_RANGE = 3.0;
const float PCL_FILTER_MAX_RANGE = 20.0;
const float PCL_FILTER_OPENING_ANGLE = M_PI/4;
const float PCL_FILTER_MIN_HEIGHT = -1.1;
const bool PCL_CONVERT2VOX = false;
const float PCL_VOXEL_SIZE = 0.2;

// DET Detection parameters
const int DET_BUFFER_SIZE = 100;
const float DET_RANGE = PCL_FILTER_MAX_RANGE;
const float DET_GRID_CELL_SIZE = 0.2;
const int DET_CELLS_PER_EDGE = int(DET_RANGE * 2 / DET_GRID_CELL_SIZE);

// TRA Tracking parameters
const int TRA_BUFFER_SIZE = 100;
const int TRA_Z_LASER = 2;
const float TRA_STD_LASPX = 0.15;	// Laser measurement noise standard deviation position1 in m
const float TRA_STD_LASPY = 0.15;	// Laser measurement noise standard deviation position2 in m
const int TRA_N_X = 5;
const int TRA_N_AUG = 7;

const float TRA_STD_A = 0.5;				// Process noise standard deviation longitudinal acceleration in m/s^2
const float TRA_STD_YAWDD = 0.25 * M_PI;	// Process noise standard deviation yaw acceleration in rad/s^2
const float TRA_LAMBDA = 9.0 - TRA_N_AUG;

// GT Ground truth parameters
const int GT_BUFFER_SIZE = 100;

// DA Data association parameters
const float DA_NN_MAX_DIST = 5.0;