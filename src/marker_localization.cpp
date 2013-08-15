/*
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_localization");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("softstroller_pose", 1);
  float data_x=0.0;
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    geometry_msgs::PoseWithCovarianceStamped push_data;
    push_data.pose.pose.position.x = data_x;
    data_x+=0.01;
    push_data.header.frame_id = "/map";
    push_data.pose.covariance[0] = 0.25;
    push_data.pose.covariance[7] = 0.25;
    push_data.pose.covariance[35] = 0.06853891945200942;

    push_data.pose.pose.orientation.z= -0.505052895004;
    push_data.pose.pose.orientation.w= 0.863088392488;

    chatter_pub.publish(push_data);
    ros::spinOnce();
    loop_rate.sleep();
   // ROS_INFO("send_data %f",(float)push_data.pose.pose.position.x);
  }

  //ros::spin();

  return 0;
}
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include <sstream>
#include <visualization_msgs/Marker.h>

ros::Publisher chatter_pub;
ros::Subscriber sub;

geometry_msgs::PoseWithCovarianceStamped push_data;

void ReceiveCallback(const visualization_msgs::Marker mrk)
{
  push_data.pose.pose.position.x = mrk.pose.position.x;
  push_data.pose.pose.position.y = mrk.pose.position.y;
  push_data.pose.pose.position.z = mrk.pose.position.z;

  push_data.pose.pose.orientation.x = mrk.pose.orientation.x;
  push_data.pose.pose.orientation.y = mrk.pose.orientation.y;
  push_data.pose.pose.orientation.z = mrk.pose.orientation.z;
  push_data.pose.pose.orientation.w = mrk.pose.orientation.w;

  ROS_INFO("get_data %f",(float)push_data.pose.pose.position.x);
  chatter_pub.publish(push_data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_localization");

  ros::NodeHandle n;

  chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("softstroller_pose", 1);
  sub = n.subscribe("visualization_marker", 1000, ReceiveCallback);
  float data_x=0.0;
  ros::Rate loop_rate(100);


  while(ros::ok())
  {
    push_data.header.frame_id = "/map";
    push_data.pose.covariance[0] = 0.25;
    push_data.pose.covariance[7] = 0.25;
    push_data.pose.covariance[35] = 0.06853891945200942;
    

   // push_data.pose.pose.orientation.z= -0.505052895004;
   // push_data.pose.pose.orientation.w= 0.863088392488;

   // chatter_pub.publish(push_data);
    ros::spinOnce();
    loop_rate.sleep();
   // ROS_INFO("send_data %f",(float)push_data.pose.pose.position.x);
  }

  //ros::spin();

  return 0;
}