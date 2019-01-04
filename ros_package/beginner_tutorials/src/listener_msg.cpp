//listener_msg.cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
//此示例展示接收message
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  //同上
  ros::init(argc, argv, "listener");
  //同上
  ros::NodeHandle n;
  //前两个参数类似Publisher, 最后一个参数表示, 收到message后, 被调用的函数
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  //ros::spin()进入循环, 调用回调函数. 它在遇到 Ctrl-C 或 master 关闭此节点时退出.  
  ros::spin();

  return 0;
}
