#ifndef LOGICALCORE_H
#define LOGICALCORE_H
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include <algorithm>
#include "mission_mode.h"
#include "point_mode.h"
#include "patrol_mode.h"
#include "shooting_mode.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Char.h>
#include "logical_core/tpos.h"
#include "tf/tf.h"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum CarState
{
  OBTAINED_GOAL_STATE,
  BUSY_STATE,
  IDLE_STATE
};

class LogicalCore
{
public:
  friend class PointMode;
  friend class PatrolMode;
  friend class ShootingMode;
  LogicalCore(tf::TransformListener &tf);
  void pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos);
  void getCarPos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos);
  void mode_callback(const std_msgs::Char::ConstPtr &pos);
  /**
   * @brief The callback function of the camera_goal publishing by armor_detect node.
   * @param msg
   */
  void getCameraGoal(const move_base_msgs::MoveBaseGoal::ConstPtr &msg);
  /**
   * @brief You can set some points in file ./param_manager/autocar_params.xml first,
   * this function will load the parameters to the vector patrol_list_.
   */
  void fillPatrolList();
  void getGoalFromPlane(const logical_core::tpos::ConstPtr &pos);
  double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
  geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg);
  /**
   * @brief stateMachine the logical core of autocar.
   */
  void stateMachine();
private:
  //We should init the patrol_list first.
  std::vector<move_base_msgs::MoveBaseGoal> patrol_list_;   //跑车需要用的数组
  std::vector<move_base_msgs::MoveBaseGoal> yaml_patrol_list_;//修改:读取部分的所有点坐标 读取list文件需要的数组
  MoveBaseClient ac_;
  tf::TransformListener &tf_;
  move_base_msgs::MoveBaseGoal goal_;
  move_base_msgs::MoveBaseGoal next_goal;
  geometry_msgs::PoseStamped old_enemy_global_goal_;
  geometry_msgs::PoseStamped enemy_goal_;
  geometry_msgs::PoseStamped enemy_global_goal_;
  geometry_msgs::PoseStamped car_position;
  ros::Subscriber send_goal_srv_;
  ros::Subscriber sub_current_pos;
  ros::Subscriber get_plane_xy;//修改：
  ros::Subscriber sub_mode;
  CarState car_state_;
  MissionMode  *p_mission_mode_;
  PointMode    *p_point_mode_;
  PatrolMode   *p_patrol_mode_;
  ShootingMode *p_shooting_mode_;
};

#endif // LOGICALCORE_H
