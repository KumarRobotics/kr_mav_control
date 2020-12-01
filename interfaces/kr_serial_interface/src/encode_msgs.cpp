#include <kr_serial_interface/comm_types.h>
#include <kr_serial_interface/encode_msgs.h>
#include <ros/console.h>

#include <boost/numeric/conversion/cast.hpp>
#include <limits>

namespace kr_mav_msgs
{
namespace
{
template <typename T>
using removeref = typename std::remove_reference<T>::type;

template <typename Target, typename Source>
removeref<Target> saturation_cast(Source const src, char const *variable_name = "")
{
  try
  {
    return boost::numeric_cast<removeref<Target>>(src);
  }
  catch(const boost::numeric::negative_overflow &e)
  {
    ROS_WARN("Negative overflow when setting %s", variable_name);
    return std::numeric_limits<removeref<Target>>::lowest();
  }
  catch(const boost::numeric::positive_overflow &e)
  {
    ROS_WARN("Positive overflow when setting %s", variable_name);
    return std::numeric_limits<removeref<Target>>::max();
  }
}
}  // namespace

// NOTE(Kartik): Macro needed in order to get the tgt variable name as a string
#define SATURATE_CAST(tgt, src) tgt = saturation_cast<decltype(tgt)>(src, #tgt)

void encodeSO3Command(const kr_mav_msgs::SO3Command &so3_command, std::vector<uint8_t> &output)
{
  struct SO3_CMD_INPUT so3_cmd_input;

  SATURATE_CAST(so3_cmd_input.force[0], so3_command.force.x * 500);
  SATURATE_CAST(so3_cmd_input.force[1], so3_command.force.y * 500);
  SATURATE_CAST(so3_cmd_input.force[2], so3_command.force.z * 500);

  SATURATE_CAST(so3_cmd_input.des_qx, so3_command.orientation.x * 125);
  SATURATE_CAST(so3_cmd_input.des_qy, so3_command.orientation.y * 125);
  SATURATE_CAST(so3_cmd_input.des_qz, so3_command.orientation.z * 125);
  SATURATE_CAST(so3_cmd_input.des_qw, so3_command.orientation.w * 125);

  SATURATE_CAST(so3_cmd_input.angvel_x, so3_command.angular_velocity.x * 1000);
  SATURATE_CAST(so3_cmd_input.angvel_y, so3_command.angular_velocity.y * 1000);
  SATURATE_CAST(so3_cmd_input.angvel_z, so3_command.angular_velocity.z * 1000);

  SATURATE_CAST(so3_cmd_input.kR[0], so3_command.kR[0] * 50);
  SATURATE_CAST(so3_cmd_input.kR[1], so3_command.kR[1] * 50);
  SATURATE_CAST(so3_cmd_input.kR[2], so3_command.kR[2] * 50);

  SATURATE_CAST(so3_cmd_input.kOm[0], so3_command.kOm[0] * 100);
  SATURATE_CAST(so3_cmd_input.kOm[1], so3_command.kOm[1] * 100);
  SATURATE_CAST(so3_cmd_input.kOm[2], so3_command.kOm[2] * 100);

  SATURATE_CAST(so3_cmd_input.cur_yaw, so3_command.aux.current_yaw * 1e4f);

  SATURATE_CAST(so3_cmd_input.kf_correction, so3_command.aux.kf_correction * 1e11f);
  SATURATE_CAST(so3_cmd_input.angle_corrections[0], so3_command.aux.angle_corrections[0] * 2500);
  SATURATE_CAST(so3_cmd_input.angle_corrections[1], so3_command.aux.angle_corrections[1] * 2500);

  so3_cmd_input.enable_motors = so3_command.aux.enable_motors;
  so3_cmd_input.use_external_yaw = so3_command.aux.use_external_yaw;

  so3_cmd_input.seq = so3_command.header.seq % 255;

  output.resize(sizeof(so3_cmd_input));
  memcpy(&output[0], &so3_cmd_input, sizeof(so3_cmd_input));
}

void encodeTRPYCommand(const kr_mav_msgs::TRPYCommand &trpy_command, std::vector<uint8_t> &output)
{
  struct TRPY_CMD trpy_cmd_input;

  SATURATE_CAST(trpy_cmd_input.thrust, trpy_command.thrust * 1e4f);
  SATURATE_CAST(trpy_cmd_input.roll, trpy_command.roll * 1e4f);
  SATURATE_CAST(trpy_cmd_input.pitch, trpy_command.pitch * 1e4f);
  SATURATE_CAST(trpy_cmd_input.yaw, trpy_command.yaw * 1e4f);
  SATURATE_CAST(trpy_cmd_input.current_yaw, trpy_command.aux.current_yaw * 1e4f);

  trpy_cmd_input.enable_motors = trpy_command.aux.enable_motors;
  trpy_cmd_input.use_external_yaw = trpy_command.aux.use_external_yaw;

  output.resize(sizeof(trpy_cmd_input));
  memcpy(&output[0], &trpy_cmd_input, sizeof(trpy_cmd_input));
}

void encodePWMCommand(const kr_mav_msgs::PWMCommand &pwm_command, std::vector<uint8_t> &output)
{
  struct PWM_CMD_INPUT pwm_cmd_input;

  SATURATE_CAST(pwm_cmd_input.pwm[0], pwm_command.pwm[0] * 255);
  SATURATE_CAST(pwm_cmd_input.pwm[1], pwm_command.pwm[1] * 255);

  output.resize(sizeof(pwm_cmd_input));
  memcpy(&output[0], &pwm_cmd_input, sizeof(pwm_cmd_input));
}
}  // namespace kr_mav_msgs
