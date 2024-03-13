#include <gtest/gtest.h>

#include <mutex>
#include <thread>

#include "kr_mav_controllers/so3_control_tester.hpp"

class SO3ControlComponentTest : public testing::Test
{
 public:
  SO3ControlComponentTest()
      : tester(std::make_shared<SO3ControlTester>()),
        executor(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
  }

  std::shared_ptr<SO3ControlTester> tester;
  std::thread spin_thread;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

  // Spinning the node in a different thread
  void SetUp() override
  {
    executor->add_node(tester);
    spin_thread = std::thread([this]() { executor->spin(); });
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // Cancel spinning of node
  void TearDown() override
  {
    executor->cancel();
    spin_thread.join();
  }
};

/*
 * @brief Test1: test if SO3Command is received without enabling any motors or odom data or position command
 * Expected behavior: so3_command_received_ = false
 * Also checking if the component is active
 */
TEST_F(SO3ControlComponentTest, Test1)
{
  ASSERT_TRUE(tester->is_so3_cmd_publisher_active());  // checking if nodelet is active
  std::cout << "Performing Test1!\n";
  {
    std::lock_guard<std::mutex> lock(tester->mutex);
    EXPECT_FALSE(tester->so3_command_received_);
  }
}

/*
 * @brief Test2: publish position command and check so3 command
 * Expected behavior: so3_command_received_ = false
 */
TEST_F(SO3ControlComponentTest, Test2)
{
  std::cout << "Performing Test2!\n";
  tester->publish_single_position_command();
  rclcpp::sleep_for(std::chrono::seconds(1));
  {
    std::lock_guard<std::mutex> lock(tester->mutex);
    EXPECT_FALSE(tester->so3_command_received_);
  }
  tester->reset_so3_cmd_pointer();
}

/*
 * @brief Test3: enable motors and publish position command. check so3 command
 * Expected behavior: so3_command_received_ = false
 */
TEST_F(SO3ControlComponentTest, Test3)
{
  tester->publish_enable_motors(true);
  tester->publish_single_position_command();
  rclcpp::sleep_for(std::chrono::seconds(1));
  {
    std::lock_guard<std::mutex> lock(tester->mutex);
    EXPECT_FALSE(tester->so3_command_received_);
  }
  tester->reset_so3_cmd_pointer();
}

/*
 * @brief Test4: enable motors, publish odom, publish position command. check so3 command
 * Expected behavior: so3_command_received_ = true
 */
TEST_F(SO3ControlComponentTest, Test4)
{
  tester->publish_enable_motors(true);
  tester->populate_odom_msgs();
  tester->publish_odom_msg(0);
  rclcpp::sleep_for(std::chrono::seconds(1));
  tester->publish_single_position_command();
  rclcpp::sleep_for(std::chrono::seconds(1));
  Test4Data ref;
  {
    std::lock_guard<std::mutex> lock(tester->mutex);
    EXPECT_TRUE(tester->so3_command_received_);
    EXPECT_EQ(tester->so3_cmd_->force.x, ref.force_x);
    EXPECT_EQ(tester->so3_cmd_->force.y, ref.force_y);
    EXPECT_EQ(tester->so3_cmd_->force.z, ref.force_z);
    EXPECT_EQ(tester->so3_cmd_->orientation.x, ref.orientation_x);
    EXPECT_EQ(tester->so3_cmd_->orientation.y, ref.orientation_y);
    EXPECT_EQ(tester->so3_cmd_->orientation.z, ref.orientation_z);
    EXPECT_EQ(tester->so3_cmd_->orientation.w, ref.orientation_w);
    EXPECT_EQ(tester->so3_cmd_->angular_velocity.x, ref.angular_velocity_x);
    EXPECT_EQ(tester->so3_cmd_->angular_velocity.y, ref.angular_velocity_y);
    EXPECT_EQ(tester->so3_cmd_->angular_velocity.z, ref.angular_velocity_z);
    EXPECT_EQ(tester->so3_cmd_->aux.current_yaw, ref.current_yaw);
    EXPECT_EQ(tester->so3_cmd_->aux.kf_correction, ref.kf_correction);
    EXPECT_EQ(tester->so3_cmd_->aux.angle_corrections[0], ref.angle_corrections[0]);
    EXPECT_EQ(tester->so3_cmd_->aux.angle_corrections[1], ref.angle_corrections[1]);
  }
  tester->reset_so3_cmd_pointer();
}

/*
 * @brief Test5: enable motors, publish odom, publish position commands (11 cmds). Check so3 command
 */
TEST_F(SO3ControlComponentTest, Test5)
{
  tester->publish_enable_motors(true);
  tester->populate_odom_msgs();
  tester->populate_position_cmd_vector(1, 1, 2, 0, 0.1);
  tester->publish_odom_msg(0);
  rclcpp::sleep_for(std::chrono::seconds(1));
  Test5Data ref;
  for(int i = 0; i < 11; i++)
  {
    tester->publish_position_command(i);
    rclcpp::sleep_for(std::chrono::seconds(1));
    {
      std::lock_guard<std::mutex> lock(tester->mutex);
      EXPECT_TRUE(tester->so3_command_received_);
      EXPECT_NEAR(tester->so3_cmd_->force.x, ref.force_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->force.y, ref.force_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->force.z, ref.force_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.x, ref.orientation_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.y, ref.orientation_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.z, ref.orientation_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.w, ref.orientation_w[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.x, ref.angular_velocity_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.y, ref.angular_velocity_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.z, ref.angular_velocity_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.current_yaw, ref.current_yaw[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.kf_correction, ref.kf_correction, 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.angle_corrections[0], ref.angle_corrections[0], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.angle_corrections[1], ref.angle_corrections[1], 1e-4);
    }
    tester->reset_so3_cmd_pointer();
  }
}

/*
 * @brief Test6: enable motors, publish odom, publish position commands (11 cmds). Check so3 command
 */
TEST_F(SO3ControlComponentTest, Test6)
{
  tester->publish_enable_motors(true);
  tester->populate_odom_msgs();
  tester->populate_position_cmd_vector(2, 3, 5, 7, 0.2);
  tester->publish_odom_msg(1);
  rclcpp::sleep_for(std::chrono::seconds(1));
  Test6Data ref;
  for(int i = 0; i < 11; i++)
  {
    tester->publish_position_command(i);
    rclcpp::sleep_for(std::chrono::seconds(1));
    {
      std::lock_guard<std::mutex> lock(tester->mutex);
      EXPECT_TRUE(tester->so3_command_received_);
      EXPECT_NEAR(tester->so3_cmd_->force.x, ref.force_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->force.y, ref.force_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->force.z, ref.force_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.x, ref.orientation_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.y, ref.orientation_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.z, ref.orientation_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.w, ref.orientation_w[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.x, ref.angular_velocity_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.y, ref.angular_velocity_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.z, ref.angular_velocity_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.current_yaw, ref.current_yaw[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.kf_correction, ref.kf_correction, 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.angle_corrections[0], ref.angle_corrections[0], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.angle_corrections[1], ref.angle_corrections[1], 1e-4);
    }
    tester->reset_so3_cmd_pointer();
  }
}

/*
 * @brief Test7: enable motors, publish corrections, publish odom, publish position commands (11 cmds). Check so3
 * command
 */
TEST_F(SO3ControlComponentTest, Test7)
{
  tester->publish_enable_motors(true);
  tester->populate_odom_msgs();
  tester->populate_position_cmd_vector(4, 2, 1, 56, 0.5);
  float kf_corr = 1.0f;
  float ang_corr[2] = {0.2f, 0.3f};
  tester->publish_corrections(kf_corr, ang_corr);
  rclcpp::sleep_for(std::chrono::seconds(1));
  tester->publish_odom_msg(2);
  rclcpp::sleep_for(std::chrono::seconds(1));
  Test7Data ref;
  for(int i = 0; i < 11; i++)
  {
    tester->publish_position_command(i);
    rclcpp::sleep_for(std::chrono::seconds(1));
    {
      std::lock_guard<std::mutex> lock(tester->mutex);
      EXPECT_TRUE(tester->so3_command_received_);
      EXPECT_NEAR(tester->so3_cmd_->force.x, ref.force_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->force.y, ref.force_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->force.z, ref.force_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.x, ref.orientation_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.y, ref.orientation_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.z, ref.orientation_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->orientation.w, ref.orientation_w[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.x, ref.angular_velocity_x[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.y, ref.angular_velocity_y[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->angular_velocity.z, ref.angular_velocity_z[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.current_yaw, ref.current_yaw[i], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.kf_correction, ref.kf_correction, 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.angle_corrections[0], ref.angle_corrections[0], 1e-4);
      EXPECT_NEAR(tester->so3_cmd_->aux.angle_corrections[1], ref.angle_corrections[1], 1e-4);
    }
    tester->reset_so3_cmd_pointer();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
