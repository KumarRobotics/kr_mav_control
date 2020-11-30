#ifndef TRACKERS_MANAGER_TRACKER_H_
#define TRACKERS_MANAGER_TRACKER_H_

#include <kr_mav_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace kr_trackers_manager
{
class Tracker
{
 public:
  virtual ~Tracker(void) {}

  /**
   * @brief Initialize the tracker. Should be used to get the params, construct the publishers and subscribers.
   *
   * @param nh The NodeHandle with the kr_trackers_manager's namespace, can be used to read common params such as gains.
   */
  virtual void Initialize(const ros::NodeHandle &nh) = 0;

  /**
   * @brief Activate the tracker. This indicates that the tracker should get ready to publish commands.
   *
   * @param cmd The last PositionCommand that was published, can be used to maintain continuity of commands when
   * switching trackers.
   *
   * @return Should return true if the tracker is ready to publish commands, else return false.
   */
  virtual bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd) = 0;

  /**
   * @brief Deactivate the tracker. This is called when the kr_trackers_manager switches to another tracker.
   */
  virtual void Deactivate(void) = 0;

  /**
   * @brief Get the current command output from the tracker.
   * Note that this function is still called even if the tracker has not been activated. This is for cases when the
   * tracker would want to use the previous robot odometry to compute current commands.
   *
   * @param msg The current odometry message which should be used by the tracker to generate the command.
   *
   * @return The PositionCommand message which would be published. If an uninitialized ConstPtr is returned, then no
   * PositionCommand message would be published.
   */
  virtual kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg) = 0;

  /**
   * @brief Get status of the tracker. Only called when the tracker has been activated.
   *
   * @return The tracker status (see the options in the TrackerStatus message).
   */
  virtual uint8_t status() const = 0;
};

}  // namespace kr_trackers_manager

#endif
