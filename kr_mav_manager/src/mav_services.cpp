#include "kr_mav_manager/mav_manager_services.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mav = std::make_shared<kr_mav_manager::MAVManager>();
    kr_mav_manager::MAVManagerServices mm_srvs(mav);
    rclcpp::spin(mav);
    rclcpp::shutdown();
    return 0;
}