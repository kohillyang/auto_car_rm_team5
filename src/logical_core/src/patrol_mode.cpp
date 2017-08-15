#include "patrol_mode.h"
#include "logical_core.h"
#include <ros/ros.h>

PatrolMode::PatrolMode(LogicalCore* logical_core)
{
  logical_core_ = logical_core;
}

void PatrolMode::obtainedGoalState()
{
  ROS_INFO("[PATROL MODE] [OBTAINED_GOAL_STATE]");
  logical_core_->ac_.sendGoal(logical_core_->goal_);
  ROS_INFO("x:%f, y=%f",logical_core_->goal_.target_pose.pose.position.x,logical_core_->goal_.target_pose.pose.position.y);//修改：
  logical_core_->car_state_ = BUSY_STATE;
  ROS_INFO("[PATROL MODE] [BUSY_STATE]");
}

void PatrolMode::busyState()
{
  //patrol model 下goal是全局的
  if (logical_core_->ac_.getState()== actionlib::SimpleClientGoalState::SUCCEEDED||
      logical_core_->distance(logical_core_->car_position, logical_core_->goal_.target_pose) < 0.5)
  {
    logical_core_->car_state_ = IDLE_STATE;
    logical_core_->patrol_list_.pop_back();
    ROS_INFO("Current goal reached，switch BUSY to IDLE_STATE!");
  }
  else if (logical_core_->ac_.getState() == actionlib::SimpleClientGoalState::LOST)
  {
    ROS_INFO("Can't make a plane!");
  }
}

void PatrolMode::idleState()
{
   
    if(!logical_core_->patrol_list_.empty())
    {
    logical_core_->goal_      = logical_core_->patrol_list_.back();//将最后一个点作为下一个目标
    logical_core_->car_state_ = OBTAINED_GOAL_STATE;
     ROS_INFO("[PATROL MODE] [IDLE_STATE]");
    }
}
