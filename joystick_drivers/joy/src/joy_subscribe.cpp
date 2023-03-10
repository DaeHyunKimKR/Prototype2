#include <unistd.h>
#include <fcntl.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>

float axes[6];
int button[8];
 void numberCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
   axes[0] = joy_msg->axes[0];
   axes[1] = joy_msg->axes[1];
   axes[2] = joy_msg->axes[2];
   axes[3] = joy_msg->axes[3];
   axes[4] = joy_msg->axes[4];
   axes[5] = joy_msg->axes[5];
   axes[6] = joy_msg->axes[6];
   axes[7] = joy_msg->axes[7];
   axes[8] = joy_msg->axes[8];
   std::cout<<axes[0]<<" , " <<axes[1]<<" , " << axes[2]<<" , " << axes[3]<<" , " << axes[4]<<" , " << axes[5] <<" , "<< axes[6]<<" , "<< axes[7]<<" , "<<axes[8]<<" , "<<  std::endl;
   //std::cout<<axes[1]<<std::endl;
   //std::cout<<axes[2]<<std::endl;
   //std::cout<<axes[3]<<std::endl;
   //std::cout<<axes[4]<<std::endl;
   for(int i = 0 ; i < 8 ; i++)
   {
     button[i] = joy_msg->buttons[i];
   }
   std::cout<<button[0]<<" , " <<button[1]<<" , " << button[2]<<" , " << button[3]<<" , " << button[4]<<" , " << button[5]<<" , " << button[6]<<" , " << button[7] <<  std::endl;
}

void subscribe()
{
    while(ros::ok())
    {
        ros::NodeHandle nh_;
        ros::Subscriber sub = nh_.subscribe("joy", 10,&numberCallback);
        ros::spin();
        //std::cout << joy_msg.axes[0]<< std::endl;
        //std::cout << joy_msg << std::endl;
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_subscribe");
  subscribe();
  return 0;
}

