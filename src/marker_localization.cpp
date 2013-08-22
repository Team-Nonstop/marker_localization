#include <sstream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include "tf/tf.h"

#include <yaml-cpp/yaml.h>
#include "marker_localization/marker_mark_data.hpp"
#include <fstream>

ros::Publisher posewcov_pub, pose_pub;
ros::Subscriber sub;

tf::TransformListener *m_tfListener;
tf::TransformBroadcaster *m_tfBroadcaster;
std::string yaml_file;

void matchParam(int marker_id, std::string &marker_frameid, std::string &target_base_link_frame_id, std::string &robot_name){
  std::ifstream ifs(yaml_file.c_str(), std::ifstream::in);

  if (ifs.good() == false)
    {
      ROS_ERROR("CmdVelMux : configuration file not found [%s]", yaml_file.c_str());
      return;
    }
  else
    ROS_WARN("YAML File found");

  YAML::Parser parser(ifs);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  for(int i = 0; i < doc.size(); i++){
      MarkerMarkData mmd;
      std::stringstream ss;
      ss << i;
      std::string robot_n = "robot_" + ss.str();

      const YAML::Node *robot_nm_node = doc[robot_n].FindValue("robot_name");
      const YAML::Node *robot_bl_node = doc[robot_n].FindValue("robot_base_link");

      mmd.configure(doc[robot_n]["mark_data"]);
      for(int j = 0; j < mmd.size(); j++){
          if(std::atoi(mmd[j].marker_id.c_str()) == marker_id){
              marker_frameid = mmd[j].marker_link;
              *robot_bl_node >> target_base_link_frame_id;
              *robot_nm_node >> robot_name;

              ROS_WARN("Marker frame id : %s\n Robot base link : %s\n Marker link : %s\n",
                  marker_frameid.c_str(), target_base_link_frame_id.c_str(), robot_name.c_str());
          }
      }
  }
}


void ReceiveCallback(const visualization_msgs::Marker mrk)
{

  std::string marker_frameid = "/softstroller/marker_front_link";
  std::string target_base_link_frame_id = "/softstroller/base_link";
  std::string map_frameid = "/map";
  std::string camera_frameid = "/watcher/camera_front";
  std::string robot_name;

  matchParam(mrk.id, marker_frameid, target_base_link_frame_id, robot_name);

  tf::StampedTransform map_to_camera_transform;
  tf::StampedTransform camera_to_marker_transform;
  tf::StampedTransform marker_to_target_base_link_transform;
  tf::Transform map_to_target_base_link_transform ;

  tf::Transform marker_transform;
  marker_transform.setOrigin( tf::Vector3(mrk.pose.position.x, mrk.pose.position.y, mrk.pose.position.z)  );
  marker_transform.setRotation( tf::Quaternion(mrk.pose.orientation.x,mrk.pose.orientation.y,mrk.pose.orientation.z,mrk.pose.orientation.w) );
  switch(mrk.id){
  // Should be replaced by due to parameter's value
  }


  try
  {

      //ROS_WARN("GET MARKERS %d",mr);
      m_tfListener->lookupTransform(map_frameid, camera_frameid, ros::Time(), map_to_camera_transform);
      m_tfListener->lookupTransform(marker_frameid, target_base_link_frame_id, ros::Time(), marker_to_target_base_link_transform);
      camera_to_marker_transform = tf::StampedTransform(marker_transform, ros::Time::now(), camera_frameid, marker_frameid);


      map_to_target_base_link_transform = map_to_camera_transform*camera_to_marker_transform*marker_to_target_base_link_transform
          ;



  }
  catch(tf::TransformException &e)
  {
      ROS_ERROR("Failed to transform");
      return;
  }



  tf::Transform transform;
  transform.setOrigin( map_to_target_base_link_transform.getOrigin() );
  transform.setRotation( map_to_target_base_link_transform.getRotation() );

  m_tfBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frameid, "/stroller"));

  //      posewcov_pub.publish(base_pose_withCov);
  //      pose_pub.publish(cam_base_pose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_localization");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  m_tfListener = new tf::TransformListener();
  m_tfBroadcaster = new tf::TransformBroadcaster();

  n.getParam("/concert/marker_localization/param/robot_config.yaml", yaml_file);

  //      posewcov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("softstroller_pose", 1);
  //      pose_pub = n.advertise<geometry_msgs::PoseStamped>("softstroller_pose_simple", 1); // For monitoring
  sub = n.subscribe("/visualization_marker", 1000, ReceiveCallback);
  //      float data_x=0.0;

  while(ros::ok())
    {
      //              base_pose_withCov.pose.covariance[0] = 0.25;
      //              base_pose_withCov.pose.covariance[7] = 0.25;
      //              base_pose_withCov.pose.covariance[35] = 0.06853891945200942;

      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
}
