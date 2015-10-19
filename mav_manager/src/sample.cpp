#include <mav_manager/mav_manager_services.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");
  ros::NodeHandle nh;

  std::shared_ptr<MAVManager> mav = std::make_shared<MAVManager>();

  MAVManagerServices mm_srvs(mav);
  
  ros::spin();

  return 0;
}
