
#include "ros/ros.h"
#include "std_msgs/Int64.h"

void pwmcallback(const std_msgs::Int64::ConstPtr& pwmmsg){
	ROS_INFO("forrden")
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "topic_listener_node");
  ros::NodeHandle n;
  ros::Subscriber pwm_sub = n.subscribe("motor_pwm", 1, pwmcallback);


  ros::Rate loop_rate(10);
  ros::spin();
  return 0;
}
