#include "logical_core.h"

LogicalCore::LogicalCore(tf::TransformListener &tf) : ac_("move_base", true), car_state_(IDLE_STATE), tf_(tf)
{
    //fillPatrolList();
    ros::NodeHandle nh_cam;
    ros::Rate loop(30);
    //car_pos        = nh_cam.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, boost::bind(&LogicalCore::getCarPos, this, _1));
    send_goal_srv_ = nh_cam.subscribe<move_base_msgs::MoveBaseGoal>("camera_goal", 10, boost::bind(&LogicalCore::getCameraGoal, this, _1));
    sub_current_pos = nh_cam.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, boost::bind(&LogicalCore::pos_callback, this, _1));
    get_plane_xy = nh_cam.subscribe<logical_core::tpos>("/Target_pose", 10, boost::bind(&LogicalCore::getGoalFromPlane, this, _1));
    sub_mode =nh_cam.subscribe<std_msgs::Char>("mode_change",5,boost::bind(&LogicalCore::mode_callback,this,_1));
    
    ROS_INFO("Preparation complete to get goals from camera.");
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started, ready for sending goals.");

    //goal_.target_pose.pose.x     = 0.0;
    //old_goal_.target_pose.pose.x = 0.0;
    //修改：预存启动区的点，让车最后能回到启动区

    p_point_mode_   = new PointMode(this);
    p_patrol_mode_  = new PatrolMode(this);
    p_shooting_mode_= new ShootingMode(this);
    p_mission_mode_ = p_patrol_mode_;

    old_enemy_global_goal_.pose.position.x = 0;
    old_enemy_global_goal_.pose.position.y = 0;

	//pose.target_pose.header.stamp = ros::Time::now();
 //蓝方
 
//红方区
 /*
	pose.target_pose.pose.position.x = -6.516;
	pose.target_pose.pose.position.y = 12.988;
	pose.target_pose.pose.orientation.z = -0.533;
	pose.target_pose.pose.orientation.w = 0.846;*/
   printf("OPEN LIST  ");
   fillPatrolList(); 
  
    while(ros::ok())
    { 

        stateMachine();
        ros::spinOnce();
        loop.sleep();
    }
}
//初始读取地图参考预测坐标数据
void LogicalCore::fillPatrolList()//修改
{
    move_base_msgs::MoveBaseGoal temp_goal;
    temp_goal.target_pose.header.frame_id = "map";

    cv::FileStorage fs("/home/ubuntu/auto_car/list.yaml", cv::FileStorage::READ);
    if( !fs.isOpened() ) // if we have file with parameters, read them
    {
        std::cout<<"ERROR, cannot open list.yaml!"<<std::endl;
    }
    cv::FileNode list_n = fs["features"];
    cv::FileNodeIterator it = list_n.begin(), it_end = list_n.end();
    std::cout << "xunluomoshi" << "  "<<std::endl;
    int i=0;
    for (; it != it_end; ++it)
    {
        temp_goal.target_pose.pose.position.x   = (double)(*it)["x"];
        temp_goal.target_pose.pose.position.y   = (double)(*it)["y"];
        temp_goal.target_pose.pose.orientation.z = (double)(*it)["z"];      
        temp_goal.target_pose.pose.orientation.w = (double)(*it)["w"];
        std::cout << i<<"  ";i++;
        std::cout << (double)(*it)["x"] << "  ";
        std::cout << (double)(*it)["y"] << std::endl;
         yaml_patrol_list_.push_back(temp_goal);
       //  patrol_list_.push_back(temp_goal);
    }
    fs.release();
}
//获取指令操作员传来的一个点并与预存坐标对比，得到最终要跑的精确点并存入patro_list_
void LogicalCore::getGoalFromPlane(const logical_core::tpos::ConstPtr &pos)//修改
{
    next_goal.target_pose.header.frame_id = "map";
    
    double planeTocar_x,planeTocar_y,planeTocar_z,planeTocar_w;
    std::cout << "7777676676"<<std::endl;
    //0.自动车获取飞机发送的x,y坐标
    //此处添加获取飞机发送的坐标的语句
    planeTocar_x  = (double)pos->x;
    planeTocar_y  = (double)pos->y;
    planeTocar_w  = (double)pos->yaw;

    tf::Quaternion a(planeTocar_w/57.3,0,0);  
    
    next_goal.target_pose.pose.position.x  = planeTocar_x;
    next_goal.target_pose.pose.position.y  = planeTocar_y;
    next_goal.target_pose.pose.orientation.z = a.z();
    next_goal.target_pose.pose.orientation.w = a.w();

    patrol_list_.push_back(next_goal);
  
}
geometry_msgs::PoseStamped LogicalCore::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
{
    std::string global_frame = "map";
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
        tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
        ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                 goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
        return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
}

double LogicalCore::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

void LogicalCore::getCameraGoal(const move_base_msgs::MoveBaseGoal::ConstPtr &msg)
{
    if(msg->target_pose.pose.position.z == 1)
    {
        enemy_goal_ = msg->target_pose;
        ROS_INFO("msg->target_pose:x=%f , y=%f ",enemy_goal_.pose.position.x,enemy_goal_.pose.position.y);
        enemy_goal_.pose.position.x -= 2.0;
        enemy_global_goal_ = goalToGlobalFrame(enemy_goal_);
        ROS_INFO("car_position:x=%f , y=%f ",car_position.pose.position.x,car_position.pose.position.y);
        
        ROS_INFO("trans_enemy_global_goal_:x=%f , y=%f ",enemy_global_goal_.pose.position.x,enemy_global_goal_.pose.position.y);
        
        //tf::Pose pose;
        //tf::poseMsgToTF(enemy_global_goal_.pose, pose);
        //double yaw = tf::getYaw(pose.getRotation());
        ROS_INFO("distance:%f",distance(enemy_global_goal_,old_enemy_global_goal_));
        if(distance(enemy_global_goal_, old_enemy_global_goal_) > 1.0)
        {
            p_mission_mode_ = p_shooting_mode_;
            //goal_.target_pose.pose.orientation.w = 1.0;
            //goal_.target_pose.pose.orientation.z = 0.0;
            car_state_ = OBTAINED_GOAL_STATE;
            old_enemy_global_goal_ = enemy_global_goal_;
            goal_.target_pose = enemy_global_goal_;
            goal_.target_pose.header.stamp = ros::Time::now();
            goal_.target_pose.pose.position.z = 2;
        }
    }
    else if(msg->target_pose.pose.position.z == 0)
    {
        if(p_mission_mode_ == p_shooting_mode_)
        {
            car_state_ = IDLE_STATE;
        }
    }
}
//消息回调函数，主函数循环中利用spinonce循环调用此函数
void LogicalCore::pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos)
{
    car_position.pose = pos->pose.pose;
    car_position.header = pos->header;
}

void LogicalCore::mode_callback(const std_msgs::Char::ConstPtr &mode)
{   
  char msg;
    msg=mode->data;
    switch (msg) 
    {
        case '0':
            patrol_list_.push_back(yaml_patrol_list_[0]);
            car_state_ = IDLE_STATE;
            ROS_INFO("receive 0");
            msg='`';
            break; 
        case '1':
            /* code for condition */
            patrol_list_.push_back(yaml_patrol_list_[1]);
            car_state_ = IDLE_STATE;
            ROS_INFO("receive 1");
            msg='`';
            break;
        case '2':
             patrol_list_.push_back(yaml_patrol_list_[2]);  
               car_state_ = IDLE_STATE;
              ROS_INFO("receive 2");
              msg='`'; 
        break;
             
        case '3':
              patrol_list_.push_back(yaml_patrol_list_[3]);
                car_state_ = IDLE_STATE;
                ROS_INFO("receive 3");
              msg='`';
         break;
       case '4':
            /* code for condition */
            patrol_list_.push_back(yaml_patrol_list_[4]);
            car_state_ = IDLE_STATE;
             ROS_INFO("receive 4");
            msg='`';
            break;
        case '5':
             patrol_list_.push_back(yaml_patrol_list_[5]);  
               car_state_ = IDLE_STATE;
               ROS_INFO("receive 5");
              msg='`'; 
        break;
             
        case '6':
              patrol_list_.push_back(yaml_patrol_list_[6]);
              car_state_ = IDLE_STATE;
               ROS_INFO("receive 6"); 
              msg='`';
         break;
        case '7':
              patrol_list_.push_back(yaml_patrol_list_[7]);
              car_state_ = IDLE_STATE;
               ROS_INFO("receive 7"); 
              msg='`';
         break;

        case '8':
              patrol_list_.push_back(yaml_patrol_list_[8]);
              car_state_ = IDLE_STATE;
               ROS_INFO("receive 8"); 
              msg='`';
         break;
        case '9':
              patrol_list_.push_back(yaml_patrol_list_[9]);
              car_state_ = IDLE_STATE;
               ROS_INFO("receive 9"); 
              msg='`';
         break;

         case 'a':
              patrol_list_.push_back(yaml_patrol_list_[10]);
              car_state_ = IDLE_STATE;
               ROS_INFO("receive a==10"); 
              msg='`';
         break;    
        case 'b':
              patrol_list_.push_back(yaml_patrol_list_[11]);
              car_state_ = IDLE_STATE;
               ROS_INFO("receiv b==11"); 
              msg='`';
         break;
        case 'c':
              patrol_list_.push_back(yaml_patrol_list_[12]);
              car_state_ = IDLE_STATE;
               ROS_INFO("receive c==12"); 
              msg='`';
         break;
        case 'd':
              patrol_list_.push_back(yaml_patrol_list_[13]);
              car_state_ = IDLE_STATE;
               ROS_INFO("receive d==13"); 
              msg='`';
         break;  

        default:
          msg='`';
          break;
            /* Default Code */
    }
    
}

void LogicalCore::stateMachine()
{
    switch(car_state_)
    {
    case OBTAINED_GOAL_STATE:
    {
        p_mission_mode_->obtainedGoalState();
        break;
    }
    case BUSY_STATE:
    {
        p_mission_mode_->busyState();
        break;
    }
    case IDLE_STATE:
    {
        p_mission_mode_->idleState();
        break;
    }
    }
}

