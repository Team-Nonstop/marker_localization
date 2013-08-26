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

<<<<<<< HEAD
ros::Publisher posewcov_pub;//, pose_pub;
=======
#include <yaml-cpp/yaml.h>
#include "marker_localization/marker_mark_data.hpp"
#include <fstream>

ros::Publisher posewcov_pub;
>>>>>>> lobster
ros::Subscriber sub;

tf::TransformListener *m_tfListener;
tf::TransformBroadcaster *m_tfBroadcaster;
<<<<<<< HEAD

geometry_msgs::PoseWithCovarianceStamped base_pose_withCov;
geometry_msgs::PoseStamped marker_pose, base_pose;
geometry_msgs::PoseStamped cam_base_pose, base_base_pose;

void getRPY(geometry_msgs::PoseStamped &pose,double &r, double &p, double &y){
	tf::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
			pose.pose.orientation.w);

	tf::Matrix3x3(quat).getRPY(r, p, y);
}

void rotateOrientation(geometry_msgs::PoseStamped &pose, double sum_yaw){
	double r, p, y;
	getRPY(pose, r, p, y);
	tf::Quaternion q = tf::createQuaternionFromRPY(r, p + sum_yaw, y);

	pose.pose.orientation.x = q.getX();
	pose.pose.orientation.y = q.getY();
	pose.pose.orientation.z = q.getZ();
	pose.pose.orientation.w = q.getW();
}

void correctOrientation(geometry_msgs::PoseStamped &pose){
	double r, p, y;
	getRPY(pose, r, p, y);
	tf::Quaternion q = tf::createQuaternionFromRPY(1.570795, p, 0);

	pose.pose.orientation.x = q.getX();
	pose.pose.orientation.y = q.getY();
	pose.pose.orientation.z = q.getZ();
	pose.pose.orientation.w = q.getW();
=======
std::string yaml_file;

geometry_msgs::PoseWithCovarianceStamped poswcov;

void matchParam(int marker_id, std::string &marker_frameid, std::string &target_base_link_frame_id, std::string &robot_name){
  std::ifstream ifs(yaml_file.c_str(), std::ifstream::in);

  if (ifs.good() == false)
    {
      ROS_ERROR("configuration file not found [%s]", yaml_file.c_str());
      return;
    }
  else
    ROS_WARN("YAML File found");

  YAML::Parser parser(ifs);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  ROS_WARN("Param config counter : %d", (int)doc.size());
  for(int i = 0; i < (int)doc.size(); i++){

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

              ROS_WARN("\n Marker frame id : %s\n Robot base link : %s\n Marker link : %s\n",
                  marker_frameid.c_str(), target_base_link_frame_id.c_str(), robot_name.c_str());
          }
      }
  }
>>>>>>> lobster
}

void ReceiveCallback(const visualization_msgs::Marker mrk)
{
<<<<<<< HEAD
	switch(mrk.id){
	case 7 : marker_pose.header.frame_id = "/softstroller/marker_front_link";
	break;
	case 8 : marker_pose.header.frame_id = "/softstroller/marker_right_link";
	break;
	case 9 : marker_pose.header.frame_id = "/softstroller/marker_backward_link";
	break;
	case 10 : marker_pose.header.frame_id = "/softstroller/marker_left_link";
	break;
	default : marker_pose.header.frame_id = "/softstroller/marker_front_link";
	break;
	}

	marker_pose.pose.orientation.w = 1;
	try
	{
		m_tfListener->transformPose("/softstroller/base_footprint", marker_pose, base_pose);

		cam_base_pose.header.frame_id = "/watcher/camera_front";
		cam_base_pose.pose.position.y = base_pose.pose.position.z; //mrk.pose.position.y + base_pose.pose.position.z;
		cam_base_pose.pose.orientation = mrk.pose.orientation;
		correctOrientation(cam_base_pose);

		switch(mrk.id){
		case 7 :
			rotateOrientation(cam_base_pose, 1.570795);
			cam_base_pose.pose.position.x = mrk.pose.position.x + base_pose.pose.position.y;
			cam_base_pose.pose.position.z = mrk.pose.position.z + base_pose.pose.position.x;
			break;
		case 8 :
			rotateOrientation(cam_base_pose, 0);
			cam_base_pose.pose.position.x = mrk.pose.position.x + base_pose.pose.position.x;
			cam_base_pose.pose.position.z = mrk.pose.position.z + base_pose.pose.position.y;
			break;
		case 9 :
			rotateOrientation(cam_base_pose, -1.570795);
			cam_base_pose.pose.position.x = mrk.pose.position.x - base_pose.pose.position.y;
			cam_base_pose.pose.position.z = mrk.pose.position.z - base_pose.pose.position.x;
			break;
		case 10 :
			rotateOrientation(cam_base_pose, 3.141592);
			cam_base_pose.pose.position.x = mrk.pose.position.x - base_pose.pose.position.x;
			cam_base_pose.pose.position.z = mrk.pose.position.z - base_pose.pose.position.y;
			break;
		default :
			rotateOrientation(cam_base_pose, 1.570795);
			cam_base_pose.pose.position.x = mrk.pose.position.x + base_pose.pose.position.y;
			cam_base_pose.pose.position.z = mrk.pose.position.z + base_pose.pose.position.x;
			break;
		}

		m_tfListener->transformPose("/watcher/base_link", cam_base_pose, base_base_pose);

	}
	catch(tf::TransformException &e)
	{
		ROS_ERROR("Failed to transform");
		return;
	}

	//base_pose_withCov.header.frame_id = base_base_pose.header.frame_id;
	base_pose_withCov.header.frame_id = "/map";
	base_pose_withCov.header.stamp = ros::Time();
	base_pose_withCov.pose.pose = base_base_pose.pose;

	/*m_tfBroadcaster->sendTransform(tf::StampedTransform(
						tf::Transform(
								tf::Quaternion(base_base_pose.pose.orientation.x, base_base_pose.pose.orientation.y,
											   base_base_pose.pose.orientation.z, base_base_pose.pose.orientation.w),
								tf::Vector3(base_base_pose.pose.position.x, base_base_pose.pose.position.y,
											base_base_pose.pose.position.z)),
								ros::Time::now(),
								"/watcher/base_link", "/watcher/softstroller_base_link"));*/

	posewcov_pub.publish(base_pose_withCov);
	//pose_pub.publish(cam_base_pose);
=======
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

  try
  {
      ROS_WARN("GET MARKERS %d",mrk.id);
      m_tfListener->lookupTransform(map_frameid, camera_frameid, ros::Time(), map_to_camera_transform);
      m_tfListener->lookupTransform(marker_frameid, target_base_link_frame_id, ros::Time(), marker_to_target_base_link_transform);
      camera_to_marker_transform = tf::StampedTransform(marker_transform, ros::Time::now(), camera_frameid, marker_frameid);

      map_to_target_base_link_transform = map_to_camera_transform*camera_to_marker_transform*marker_to_target_base_link_transform;
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

  //posewcov_pub.publish(base_pose_withCov);
>>>>>>> lobster
}

int main(int argc, char** argv)
{
<<<<<<< HEAD
	ros::init(argc, argv, "marker_localization");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	m_tfListener = new tf::TransformListener();
	m_tfBroadcaster = new tf::TransformBroadcaster();

	posewcov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("softstroller_pose", 1);
	//pose_pub = n.advertise<geometry_msgs::PoseStamped>("softstroller_pose_simple", 1); // For monitoring
	sub = n.subscribe("visualization_marker", 1000, ReceiveCallback);
	float data_x=0.0;

	while(ros::ok())
	{
		base_pose_withCov.pose.covariance[0] = 0.25;
		base_pose_withCov.pose.covariance[7] = 0.25;
		base_pose_withCov.pose.covariance[35] = 0.06853891945200942;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
=======
  ros::init(argc, argv, "marker_localization");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  m_tfListener = new tf::TransformListener();
  m_tfBroadcaster = new tf::TransformBroadcaster();


  n.getParam("/concert/marker_localization/robot_config_file", yaml_file);

  posewcov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("softstroller_pose", 1);
  sub = n.subscribe("/visualization_marker", 1000, ReceiveCallback);

  /*std::string str;
  matchParam(0, str, str, str);*/

  while(ros::ok())
    {
      //base_pose_withCov.pose.covariance[0] = 0.25;
      //base_pose_withCov.pose.covariance[7] = 0.25;
      //base_pose_withCov.pose.covariance[35] = 0.06853891945200942;
      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
>>>>>>> lobster
}
