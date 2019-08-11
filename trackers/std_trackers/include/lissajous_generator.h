#ifndef _LISSAJOUS_GENERATOR_H_
#define _LISSAJOUS_GENERATOR_H_

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tracker_msgs/TrackerStatus.h>
#include <tracker_msgs/LissajousTrackerAction.h>
#include <tracker_msgs/LissajousAdderAction.h>
#include <actionlib/client/simple_action_client.h>

class LissajousGenerator
{
  public:
    LissajousGenerator(void);
    void setParams(const tracker_msgs::LissajousTrackerGoal::ConstPtr &msg);
    void setParams(const tracker_msgs::LissajousAdderGoal::ConstPtr &msg, int num);
    const quadrotor_msgs::PositionCommand::Ptr getPositionCmd(void);
    bool activate(void);
    void deactivate(void);
    bool isActive(void);
    bool goalIsSet(void);
    bool status(void) const;
    float timeRemaining(void);
    float timeElapsed(void);

  private:
    double lissajous_period_, ramp_time_, total_time_, ramp_s_, total_s_, const_time_, period_;
    double x_amp_, y_amp_, z_amp_, yaw_amp_;
    double x_num_periods_, y_num_periods_, z_num_periods_, yaw_num_periods_;
    double a7_, a6_, a5_, a4_;
    double num_cycles_;
    bool active_, goal_set_, goal_reached_;
    ros::Time start_time_;
};

#endif
