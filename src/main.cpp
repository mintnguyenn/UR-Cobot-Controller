#include <ros/ros.h>
#include <time.h>

#include "ur5e_controller.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "manipulator_controller");

  ros::NodeHandle nh;

  /////////////////////////////////////////// TESTING PRM STAR ALGORITHM //////////////////////////////////////////

  // std::vector<double> def{0, -M_PI / 2, 0, -M_PI / 2, 0, 0};
  // std::vector<double> start{0, -M_PI, 0, -M_PI / 2, 0, 0};

  // controller::linearInterpolateConfigurationss(joint_result);

  std::shared_ptr<Manipulator_Controller> controller(new Manipulator_Controller());
  // controller->trajectoryFromArray(joint_result);




  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}
