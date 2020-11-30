#include <kr_serial_interface/comm_types.h>
#include <kr_serial_interface/decode_msgs.h>

#include <Eigen/Geometry>

namespace kr_mav_msgs
{
bool decodeOutputData(const std::vector<uint8_t> &data, kr_mav_msgs::OutputData &output)
{
  struct OUTPUT_DATA output_data;
  if(data.size() != sizeof(output_data))
    return false;

  memcpy(&output_data, &data[0], sizeof(output_data));
  output.loop_rate = output_data.loop_rate;
  output.voltage = output_data.voltage / 1e3f;

  const double roll = output_data.roll / 1e2f * M_PI / 180;
  const double pitch = output_data.pitch / 1e2f * M_PI / 180;
  const double yaw = output_data.yaw / 1e2f * M_PI / 180;
  // Asctec (2012 firmware) uses  Z-Y-X convention
  Eigen::Quaternionf q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
  output.orientation.w = q.w();
  output.orientation.x = q.x();
  output.orientation.y = q.y();
  output.orientation.z = q.z();

  output.angular_velocity.x = output_data.ang_vel[0] * 0.0154f * M_PI / 180;
  output.angular_velocity.y = output_data.ang_vel[1] * 0.0154f * M_PI / 180;
  output.angular_velocity.z = output_data.ang_vel[2] * 0.0154f * M_PI / 180;

  output.linear_acceleration.x = output_data.acc[0] / 1e3f * 9.81f;
  output.linear_acceleration.y = output_data.acc[1] / 1e3f * 9.81f;
  output.linear_acceleration.z = output_data.acc[2] / 1e3f * 9.81f;

  output.pressure_dheight = output_data.dheight / 1e3f;
  output.pressure_height = output_data.height / 1e3f;

  output.magnetic_field.x = output_data.mag[0] / 2500.0f;
  output.magnetic_field.y = output_data.mag[1] / 2500.0f;
  output.magnetic_field.z = output_data.mag[2] / 2500.0f;

  for(int i = 0; i < 8; i++)
  {
    output.radio_channel[i] = output_data.radio[i];
  }

  // Asctec firmware uses the following rotor numbering convention:
  //   *1*    Front
  // 3     4
  //    2
  //
  // But we want:
  //   *1*    Front
  // 2     4
  //    3
  const int motors_map[] = {0, 2, 1, 3};
  for(int i = 0; i < 4; i++)
  {
    // The following conversion is from
    // http://wiki.asctec.de/display/AR/List+of+all+predefined+variables%2C+commands+and+parameters
    //
    // motorRPM = 1075+m*37.625
    //
    // Note: If m == 0, the motors are not spinning.
    int m = output_data.rpm[motors_map[i]];
    output.motor_rpm[i] = (m == 0 ? 0 : 1075) + m * 37.625;
  }

  output.seq = output_data.seq;

  return true;
}

bool decodeStatusData(const std::vector<uint8_t> &data, kr_mav_msgs::StatusData &status)
{
  struct STATUS_DATA status_data;
  if(data.size() != sizeof(status_data))
    return false;
  memcpy(&status_data, &data[0], sizeof(status_data));

  status.loop_rate = status_data.loop_rate;
  status.voltage = status_data.voltage / 1e3f;
  status.seq = status_data.seq;

  return true;
}

}  // namespace kr_mav_msgs
