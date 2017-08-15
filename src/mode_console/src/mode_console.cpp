#include "ros/ros.h"
#include "std_msgs/Char.h"
#include <sstream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <tf/tf.h>
using namespace std;
 
std_msgs::Char   mode;
ros::Publisher mode_pub;

class map_show{
    
  public:
   map_show();
   ros::NodeHandle nh;
   ros::Subscriber  Map_printf;
  void map_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& map_data);

};
map_show::map_show()
{
  Map_printf=nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 50, &map_show::map_callback,this);
  printf("kaishi\r\n");


}
 void map_show::map_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& map_data)
{
        // printf("shoudao\r\n");
 double yaw=tf::getYaw(map_data->pose.pose.orientation);
 cout<<"now position  x="<< map_data->pose.pose.position.x<<"   y= " \
 << map_data->pose.pose.position.y \
 << "  yaw=" <<yaw*57.3<<endl ;

}

//point  定点模式就是补单模式
//patrol   巡逻模式
//shoot   射击模式

void kbhit()  
{  
    struct termios oldt, newt;  
    char  ch;
    int oldf;  
    tcgetattr(STDIN_FILENO, &oldt);  
    newt = oldt;  
    newt.c_lflag &= ~(ICANON | ECHO);  
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
    ch=getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
    fcntl(STDIN_FILENO, F_SETFL, oldf); 
     
    if(ch=='0'||ch=='1'||ch=='2'||ch=='3'||ch=='4'||ch=='5'||ch=='6'||
     ch=='7'||ch=='8'||ch=='9'||ch=='a'||ch=='b'||ch=='c'||ch=='d')  
    {  

        printf("You entered mode %c:\r\n", ch);
        printf("\nPlease enter your comment:\n");
        //ungetc(ch, stdin);
        mode.data=ch;
		mode_pub.publish(mode);
    }
   
}



int main(int argc, char *argv[])
{
   
    ros::init(argc, argv, "mode_console");
   
    ros::NodeHandle n;
    
     mode_pub = n.advertise<std_msgs::Char>("mode_change", 10); //boost::bind(&void::map_callback,this,_1)

    ros::Rate loop_rate(10);
     map_show map;

    printf("Please enter your comment:\n");
    while(n.ok())
    {
        ros::spinOnce();
        kbhit();
        
        loop_rate.sleep();
    }


    return 0;
}


