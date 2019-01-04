//talker.cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
//此程序演示简单的ROS系统消息发送
int main(int argc, char **argv)
{
  //使用ROS系统任何部分之前, 必须先调用ros::init()
  //ros::init接收argc和argv参数, 这样它就可以从命令行接收任何ROS参数了; 第三个参数是`节点的名字`
  ros::init(argc, argv, "talker");
  //NodeHandle是与ROS系统通信的主要访问点. 第一个被构造的NodeHandle将完全初始化此节点
  //最后一个被析构的NodeHandle将关闭此节点
  ros::NodeHandle n;
  //advertise()函数告诉ROS你要如何在给定的topic名上发布, 这会引发对ROS master node的调用
  //master node管理谁在publishing和谁在subscribing, 调用advertise()之后, master node
  //将通知subscribe此topic name的节点, 然后它们将进行一对一的协商(negotiate).
  //advertise()返回一个Publisher对象, 可以通过调用publish()在此topic上publish messages
  //一旦所有被返回的Publisher对象被销毁, 此topic将自动unadvertised
  //advertise()的第二个参数为用来发布消息的message queue的大小, 如果messages发布的比
  //能发送的快, 此参数就表示缓存多少信息后开始舍弃多余的信息
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  //printf/cout的替代
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    //message对象, 用数据填充它, 然后发布  
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    //publish()发送messages, 参数为message对象. 参数类型必须与模板函数advertise<>()的
     //一致模板参数
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
