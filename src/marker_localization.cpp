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

ros::Publisher posewcov_pub, pose_pub;
ros::Subscriber sub;

tf::TransformListener *m_tfListener;
tf::TransformBroadcaster *m_tfBroadcaster;


void ReceiveCallback(const visualization_msgs::Marker mrk)
{

  std::string marker_frameid = "/softstroller/marker_front_link";
  std::string target_base_link_frame_id = "/softstroller/base_link";
  std::string map_frameid = "/map";
  std::string camera_frameid = "/watcher/camera_front";
  
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

ROS_WARN("GET MARKERS %d",mr);
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

//	posewcov_pub.publish(base_pose_withCov);
//	pose_pub.publish(cam_base_pose);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "marker_localization");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	m_tfListener = new tf::TransformListener();
	m_tfBroadcaster = new tf::TransformBroadcaster();

//	posewcov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("softstroller_pose", 1);
//	pose_pub = n.advertise<geometry_msgs::PoseStamped>("softstroller_pose_simple", 1); // For monitoring
	sub = n.subscribe("/visualization_marker", 1000, ReceiveCallback);
//	float data_x=0.0;

	while(ros::ok())
	{
//		base_pose_withCov.pose.covariance[0] = 0.25;
//		base_pose_withCov.pose.covariance[7] = 0.25;
//		base_pose_withCov.pose.covariance[35] = 0.06853891945200942;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
