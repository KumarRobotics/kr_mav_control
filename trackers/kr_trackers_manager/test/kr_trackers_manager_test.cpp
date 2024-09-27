#include <gtest/gtest.h>
#include <ros/ros.h>

#include <mutex>
#include <thread>

#include "kr_trackers_manager/tester_utils.hpp"

/*
 * @brief Test1: test if the tester connects properly with the trackers_manager.
 */
TEST(TrackersManagerTest, Test1)
{
  TrackersManagerTester tester;
  ASSERT_TRUE(tester.initial_checks());
}

/*
 * @brief Test2: test if any tracker is found or activated using the transition service
 */
TEST(TrackersManagerTest, Test2)
{
  TrackersManagerTester tester;
  Test2Data ref;
  std::string tracker_names[3] = {"LineTrackerDistance", "kr_trackers/LineTrackerDistance",
                                  "kr_trackers/LineTrackerMinJerk"};
  for(int i = 0; i < 3; i++)
  {
    tester.send_transition_request(tracker_names[i]);
    ros::Duration(0.5).sleep();
    {
      std::lock_guard<std::mutex> lock(tester.mutex);
      ASSERT_TRUE(tester.srv_succeed);
      EXPECT_EQ(tester.srv_response, ref.srv_response_success[i]);
      EXPECT_EQ(tester.srv_msg, ref.srv_response_msg[i]);
    }
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
