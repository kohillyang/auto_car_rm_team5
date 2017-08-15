//该定点模式就是补蛋模式



#include "point_mode.h"
#include "logical_core.h"

PointMode::PointMode(LogicalCore *logical_core)
{
  MissionMode();
  logical_core_ = logical_core;
  drone_state.data=0;
}


//先回到补蛋区
void PointMode::obtainedGoalState()   
{
  ROS_INFO("Go to budanqu under POINT_MODE");
 // logical_core_->ac_.sendGoal(logical_core_->goal_);
    move_base_msgs::MoveBaseGoal pose;
       pose.target_pose.header.frame_id = "map";
	pose.target_pose.header.stamp = ros::Time::now();
	pose.target_pose.pose.position.x = 99.0;
	pose.target_pose.pose.position.y = 104.0;
	pose.target_pose.pose.orientation.z = -0.64;
	pose.target_pose.pose.orientation.w = 0.77;
  send_goal_pose(pose);
  logical_core_->car_state_ = BUSY_STATE;
  ROS_INFO("Switch to BUSY_STATE");
}

//向无人机发送补蛋指令 补蛋
void PointMode::busyState()
{
  /*
    ros::NodeHandle budan;  
    ros::Publisher car_state_pub = budan.advertise<std_msgs::Char>("/rmg2/car_state", 10);
    ros::Subscriber drone_state_sub = budan.subscribe<std_msgs::Char>("/rmg2/drone_state", 10,boost::bind(&PointMode::drone_state_callback, this, _1)); 
   
   //车上补单机构打开信号
    ros::Publisher machine_pub = budan.advertise<std_msgs::Char>("/rmg2/machine", 10); 
    std_msgs::Char ma;
    ma.data=92; //  (11000000) youwenti
    machine_pub.publish(ma);

    std_msgs::Char bu;
    bu.data=63; //  (10100011)  youwenti
    car_state_pub.publish(bu);

   while((drone_state.data & 0x08) ==0)   //飞机没有对的反馈数据
    {  
       car_state_pub.publish(ma);
       ros::Duration(10);
    }

  
    drone_state.data=0;
  while((drone_state.data & 0x0c) ==0)   //等待飞机完成补单
  {
     ros::Duration(1);
  }
   
   //关闭补单机构
    ma.data=3; //  (00000011)
    machine_pub.publish(ma);
  */
   ROS_INFO("Point goal reached，switch to IDLE_STATE!");
   logical_core_->car_state_ = IDLE_STATE;

}

void PointMode::idleState()
{
  ROS_INFO("Switch to the PATROL_MODE!");
  logical_core_->p_mission_mode_ = logical_core_->p_patrol_mode_;
}


void PointMode::send_goal_pose(move_base_msgs::MoveBaseGoal pose)
{
	MoveBaseClient ac("move_base", true);
	// Wait 60 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer();
	ROS_INFO("Connected to move base server");
	ROS_INFO("Sending goal");
	ac.sendGoal(pose);
	// Wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30));
    if(finished_before_timeout)	
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished,state:%s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish.");
    }
}
 void PointMode::drone_state_callback(const std_msgs::Char::ConstPtr& msg)
{

  char sum,tmp;
  char  state;
  state=msg->data;
  

  tmp=0;
  //开始校验  
    for(int i=0;i<6;i++)
    { 
      
       if((state<<i)>0)
       {
          tmp=tmp+1;

       }

    }
    sum= 0x03 & (msg->data);
   if(sum==tmp)
   {
     //校验通过
     drone_state.data=msg->data;

   }

}