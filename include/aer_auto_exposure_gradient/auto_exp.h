#ifndef EXP_ROS_NODE_H_
#define EXP_ROS_NODE_H_

#include <cmath>
#include <fstream>
#include <iostream>
#include <libgen.h>
#include <math.h>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <iomanip>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/GroupState.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

//#include <aer_auto_exposure_gradient/Dehaze.h>



namespace exp_node {

class ExpNode {
 public:
  ExpNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);


 private:

  void CameraCb(const sensor_msgs::ImageConstPtr &msg);
  double image_gradient_gamma(cv::Mat &src_img, int j);
  void ChangeParam (double shutter_new, double gain_new);
  
  double * curveFit (double x[7], double y[7]);
  double  findRoots1 (double a[6], double check);
  void generate_LUT ();
  bool check_rate = false;
  double frame_rate_req = 10.0; // maximum 80 fps



 double gamma[7]={1.0/1.9, 1.0/1.5, 1.0/1.2, 1.0, 1.2, 1.5, 1.9};
 double metric[7];
 double max_metric;
 double max_gamma,alpha, expNew, expCur, shutter_cur, shutter_new, gain_cur, gain_new,upper_shutter;
 double lower_shutter = 100.0; // adjust if necessary [unit: micro-second]
 double kp=0.4; // contorl the speed to convergence
 double d = 0.1, R; // parameters used in the nonliear function in Shim's 2018 paper 				
 int gamma_index; // index to record the location of the optimum gamma value
 bool gain_flag = false;
 std::string image_topic ="camera/image_raw";



// Whether or not turn on the dehaze mode (Dehazing method: Kaiming He, 2011)
<<<<<<< HEAD
bool dehaze_mode = false;
=======
bool dehaze_mode = true;
>>>>>>> 5d2d7e1981c668d298041a72a6e7044719f41b01
int filter_size = 5; // Filter size of the filters used in the dehazing algorithm



// Parameters that correlated to Shim's Gradient Metric
double met_act_thresh = 0.06;
double lamda = 1000.0; // The lamda value used in Shim's 2014 paper as a control parameter to adjust the mapping tendency (larger->steeper) 



  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_camera_; 




};

}

#endif
