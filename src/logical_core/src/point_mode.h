#ifndef POINTMODE_H
#define POINTMODE_H
#include "mission_mode.h"
#include "ros/ros.h"
#include "std_msgs/Char.h"

class LogicalCore;
class PointMode: public MissionMode
{
public:
  PointMode(LogicalCore *logical_core);
  void obtainedGoalState();
  void busyState();
  void idleState();
  void send_goal_pose(move_base_msgs::MoveBaseGoal pose);
   void drone_state_callback(const std_msgs::Char::ConstPtr& msg);
private:
  LogicalCore *logical_core_;
  
  std_msgs::Char drone_state;
};

#endif // POINTMODE_H

