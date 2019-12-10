#include <ros/ros.h>
#include <ros/console.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
bool DEBUG = true;

// publisher and subscriber objects
ros::Publisher wheel_pub;
ros::Subscriber action_sub;
ros::Publisher steer_pub;
ros::Publisher steer_pub2;
geometry_msgs::Twist wheel_msg;
std_msgs::Float64 steer_msg;
std_msgs::Float64 steer_msg2;


void action_callback(const std_msgs::Int64::ConstPtr &action_msg)
// get new action, translate to pins and publish
{
  // unpack the message
  int action = action_msg->data;
  static double steerdata;
  //wheel_msg.angular.z = 0;
  //steer_msg = 0;
  if(action == 4){
    wheel_msg.linear.x = 0;
    steer_msg.data = steerdata; // Since these are revolute joints and i now use diffdrive 
                           // If this was angular they would have rotaded in different directions = BAD!
  }
  else if(action>=6 && action<= 8){
    wheel_msg.linear.x = 1;
    if(action == 6 && steer_msg.data > -0.7){
     steer_msg.data = steer_msg.data-0.07;
    }
    else if(action == 8 && steer_msg.data < 0.7){
     steer_msg.data = steer_msg.data+0.07;
    }
    else {steer_msg.data = 0;}
  }
  else if(action>=0 && action<=2){
    wheel_msg.linear.x = -1;
    if(action == 0 && steer_msg.data > -0.7) {
      steer_msg.data = steer_msg.data-0.07;
    }
    else if(action == 2 && steer_msg.data < 0.7) {
      steer_msg.data = steer_msg.data+0.07;
    }
    else {steer_msg.data = 0;}
  }
  else{
    wheel_msg.angular.z = 0;
    wheel_msg.linear.x = 0;
    steer_msg.data = steer_msg.data;
  }
  
  steer_msg2.data=-steer_msg.data;
  // publish pins to arduino
  wheel_pub.publish(wheel_msg);
  steer_pub.publish(steer_msg);
  steer_pub2.publish(steer_msg2);
  steerdata = steer_msg.data;


  // Debug msgs
  if(DEBUG)
  {
    std::string action_list[] = {"Backward Left", "Backward", "Backward Right",
                                 "Left",          "Idle",     "Right",
                                 "Forward Left",  "Forward",  "Forward Right"};
    ROS_INFO("Received action:");
    ROS_INFO_STREAM(action_list[action]);
    ROS_INFO_STREAM(action);
  }
}


int main(int argc, char **argv){
  // init node and subs/pubs
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle n;
  action_sub = n.subscribe("motor_action", 1, action_callback);   // read from topic "motor_action"
  steer_pub = n.advertise<std_msgs::Float64>("joint_position_controller/command",1);
  steer_pub2 = n.advertise<std_msgs::Float64>("joint_position_controller2/command",1);
  wheel_pub = n.advertise<geometry_msgs::Twist>("rig_vel_controller/cmd_vel", 1);     // pin_pub is a publisher to topic "pins" -> make it possible for controller.ino to read "pins" -> control the motors
 

  // Loop and checks all new messages in the subscribed topics above
  ros::Rate loop_rate(10);
  ros::spin();
  return 0;
}

