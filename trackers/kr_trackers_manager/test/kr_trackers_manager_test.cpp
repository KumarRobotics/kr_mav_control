#include <gtest/gtest.h>
#include <ros/ros.h>

#include <mutex>
#include <thread>

#include "kr_trackers_manager/tester_utils.hpp"

/*
 * @brief Test1: test if the tester connects properly with the trackers_manager.
 */
TEST(TrackersManagerTest, InitializationChecks)
{
  TrackersManagerTester tester;
  ASSERT_TRUE(tester.initial_checks());
}

/*
 * @brief Test2: test if any tracker is found or activated using the transition service.
          No tracker should be activated without any prior odom or goal messages.
 */
TEST(TrackersManagerTest, TrackerTransitionCheck)
{
  TrackersManagerTester tester;
  Test2Data data;
  std::string tracker_names[3] = {"LineTrackerDistance", "kr_trackers/LineTrackerDistance",
                                  "kr_trackers/LineTrackerMinJerk"};
  for(int i = 0; i < 3; i++)
  {
    tester.send_transition_request(tracker_names[i]);
    ros::Duration(0.5).sleep();
    {
      std::lock_guard<std::mutex> lock(tester.mutex);
      ASSERT_TRUE(tester.srv_succeed);
      EXPECT_EQ(tester.srv_response, data.srv_response_success[i]);
      EXPECT_EQ(tester.srv_msg, data.srv_response_msg[i]);
    }
    tester.reset_flags();
  }
}

/*
 * @brief Test3: Send a goal, activate tracker and send odom messages to reach goal.
 *        A value of -100 indicates no need to check that value/index.
 *        This test is solely to tests the action client and server communication.
 *        1. Send a goal
 *        2. Send a odom message
 *        3. Send a tracker transition request
 *        4. Send multiple odom messages until goal is reached.
 */
TEST(TrackersManagerTest, GoalCompletionCheck)
{
  TrackersManagerTester tester;
  Test3Data data;
  std::string tracker_name = "kr_trackers/LineTrackerDistance";
  ros::Duration(1.0).sleep();
  tester.send_action_goal(tracker_name, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
  ros::Duration(0.5).sleep();
  tester.publish_odom_msg(0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ros::Duration(0.5).sleep();
  tester.send_transition_request(tracker_name);
  ros::Duration(0.5).sleep();
  {
    std::lock_guard<std::mutex> lock(tester.mutex);
    ASSERT_TRUE(tester.srv_succeed);
    EXPECT_TRUE(tester.srv_response);
  }
  int num_samples = 7;
  for(int i = 0; i < num_samples; i++)
  {
    tester.publish_odom_msg(data.odom_secs[i], data.odom_nsecs[i], data.odom_pos_x[i], data.odom_pos_y[i],
                            data.odom_pos_z[i], data.odom_orient_x[i], data.odom_orient_y[i], data.odom_orient_z[i],
                            data.odom_orient_w[i]);
    ros::Duration(1.0).sleep();
    {
      std::lock_guard<std::mutex> lock(tester.mutex);

      ASSERT_TRUE(tester.position_cmd_received);
      EXPECT_NEAR(tester.cmd->position.x, data.cmd_pos_x[i], 1e-4);
      EXPECT_NEAR(tester.cmd->position.y, data.cmd_pos_y[i], 1e-4);
      EXPECT_NEAR(tester.cmd->position.z, data.cmd_pos_z[i], 1e-4);
      EXPECT_NEAR(tester.cmd->velocity.x, data.cmd_vel_x[i], 1e-4);
      EXPECT_NEAR(tester.cmd->velocity.y, data.cmd_vel_y[i], 1e-4);
      EXPECT_NEAR(tester.cmd->velocity.z, data.cmd_vel_z[i], 1e-4);
      EXPECT_NEAR(tester.cmd->acceleration.x, data.cmd_accel_x[i], 1e-4);
      EXPECT_NEAR(tester.cmd->acceleration.y, data.cmd_accel_y[i], 1e-4);
      EXPECT_NEAR(tester.cmd->acceleration.z, data.cmd_accel_z[i], 1e-4);
      EXPECT_NEAR(tester.cmd->jerk.x, data.cmd_jerk_x[i], 1e-4);
      EXPECT_NEAR(tester.cmd->jerk.y, data.cmd_jerk_y[i], 1e-4);
      EXPECT_NEAR(tester.cmd->jerk.z, data.cmd_jerk_z[i], 1e-4);
      EXPECT_NEAR(tester.cmd->yaw, data.cmd_yaw[i], 1e-4);
      EXPECT_NEAR(tester.cmd->yaw_dot, data.cmd_yawdot[i], 1e-4);

      if(data.feedback[i] != -100.0)
      {
        ASSERT_TRUE(tester.feedback_received);
        EXPECT_NEAR(tester.action_feedback->distance_from_goal, data.feedback[i], 1e-4);
      }

      EXPECT_EQ(tester.status->tracker, data.status_tracker[i]);
      EXPECT_EQ(tester.status->status, data.status_status[i]);

      if(i == num_samples - 1)
      {
        EXPECT_EQ(tester.action_result->x, data.result_x);
        EXPECT_EQ(tester.action_result->y, data.result_y);
        EXPECT_EQ(tester.action_result->z, data.result_z);
        EXPECT_EQ(tester.action_result->yaw, data.result_yaw);
        EXPECT_EQ(tester.action_result->length, data.result_length);
        EXPECT_NEAR(tester.action_result->duration, data.result_duration, 1e-4);
      }
    }
    tester.reset_flags();
  }
}

/*
 * @brief Test4: Send a goal during an active goal and see if the first goal is cancelled new goal accepted.
 *        A value of -100 indicates no need to check that value/index.
 *        This test is solely to tests the action client and server communication.
 *        1. Send a goal
 *        2. Send some odom messages
 *        3. Send another foal
 *        4. Send odom messages until new goal is reached.
 */
TEST(TrackersManagerTest, GoalPreEmptionCheck)
{
  TrackersManagerTester tester;
  Test4Data data;
  std::string tracker_name = "kr_trackers/LineTrackerDistance";
  ros::Duration(1.0).sleep();
  tester.send_action_goal(tracker_name, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
  ros::Duration(0.5).sleep();
  int num_samples = 7;
  for(int i = 0; i < num_samples; i++)
  {
    // sending a new goal while a goal is active
    if(i == 4)
    {
      tester.send_action_goal(tracker_name, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
      ros::Duration(0.5).sleep();
      continue;
    }

    if(data.odom_secs[i] != -100)
    {
      tester.publish_odom_msg(data.odom_secs[i], data.odom_nsecs[i], data.odom_pos_x[i], data.odom_pos_y[i],
                              data.odom_pos_z[i], data.odom_orient_x[i], data.odom_orient_y[i], data.odom_orient_z[i],
                              data.odom_orient_w[i]);
      ros::Duration(1.2).sleep();
    }

    {
      std::lock_guard<std::mutex> lock(tester.mutex);

      if(data.cmd_pos_x[i] != -100.0)
      {
        ASSERT_TRUE(tester.position_cmd_received);
        EXPECT_NEAR(tester.cmd->position.x, data.cmd_pos_x[i], 1e-4);
        EXPECT_NEAR(tester.cmd->position.y, data.cmd_pos_y[i], 1e-4);
        EXPECT_NEAR(tester.cmd->position.z, data.cmd_pos_z[i], 1e-4);
        EXPECT_NEAR(tester.cmd->velocity.x, data.cmd_vel_x[i], 1e-4);
        EXPECT_NEAR(tester.cmd->velocity.y, data.cmd_vel_y[i], 1e-4);
        EXPECT_NEAR(tester.cmd->velocity.z, data.cmd_vel_z[i], 1e-4);
        EXPECT_NEAR(tester.cmd->acceleration.x, data.cmd_accel_x[i], 1e-4);
        EXPECT_NEAR(tester.cmd->acceleration.y, data.cmd_accel_y[i], 1e-4);
        EXPECT_NEAR(tester.cmd->acceleration.z, data.cmd_accel_z[i], 1e-4);
        EXPECT_NEAR(tester.cmd->jerk.x, data.cmd_jerk_x[i], 1e-4);
        EXPECT_NEAR(tester.cmd->jerk.y, data.cmd_jerk_y[i], 1e-4);
        EXPECT_NEAR(tester.cmd->jerk.z, data.cmd_jerk_z[i], 1e-4);
        EXPECT_NEAR(tester.cmd->yaw, data.cmd_yaw[i], 1e-4);
        EXPECT_NEAR(tester.cmd->yaw_dot, data.cmd_yawdot[i], 1e-4);
      }

      if(data.feedback[i] != -100.0)
      {
        ASSERT_TRUE(tester.feedback_received);
        EXPECT_NEAR(tester.action_feedback->distance_from_goal, data.feedback[i], 1e-4);
      }

      if(data.status_tracker[i] != std::string("NoCheck"))
      {
        EXPECT_EQ(tester.status->tracker, data.status_tracker[i]);
        EXPECT_EQ(tester.status->status, data.status_status[i]);
      }

      if(i == 4)
      {
        EXPECT_EQ(tester.action_result->x, 0.0);
        EXPECT_EQ(tester.action_result->y, 0.0);
        EXPECT_EQ(tester.action_result->z, 0.0);
        EXPECT_EQ(tester.action_result->yaw, 0.0);
        EXPECT_EQ(tester.action_result->length, 0.0);
        EXPECT_NEAR(tester.action_result->duration, 0.0, 1e-4);
      }
      else if(i == num_samples - 1)
      {
        EXPECT_EQ(tester.action_result->x, data.result_x);
        EXPECT_EQ(tester.action_result->y, data.result_y);
        EXPECT_EQ(tester.action_result->z, data.result_z);
        EXPECT_EQ(tester.action_result->yaw, data.result_yaw);
        EXPECT_NEAR(tester.action_result->length, data.result_length, 1e-4);
        EXPECT_NEAR(tester.action_result->duration, data.result_duration, 1e-4);
      }
    }
    tester.reset_flags();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kr_trackers_manager_tester");
  testing::InitGoogleTest(&argc, argv);

  std::thread t(
      []
      {
        while(ros::ok())
          ros::spin();
      });

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  t.join();

  return res;
}
