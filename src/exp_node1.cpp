#include "aer_auto_exposure_gradient/auto_exp.h"
#include "aer_auto_exposure_gradient/Dehaze.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "exp_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    exp_node::ExpNode exp_node1(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
