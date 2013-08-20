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

ros::Publisher posewcov_pub;//, pose_pub;
ros::Subscriber sub;

tf::TransformListener *m_tfListener;
tf::TransformBroadcaster *m_tfBroadcaster;

geometry_msgs::PoseWithCovarianceStamped base_pose_withCov;
geometry_msgs::PoseStamped marker_pose, base_pose;
geometry_msgs::PoseStamped cam_base_pose, base_base_pose;

void rotateOrientation(geometry_msgs::PoseStamped &pose, double sum_pitch){
	double r, p, y;

	tf::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
			pose.pose.orientation.w);

	tf::Matrix3x3(quat).getRPY(r, p, y);

	pose.pose.orientation.x = tf::createQuaternionFromRPY(0, p + sum_pitch, 0).getX();
	pose.pose.orientation.y = tf::createQuaternionFromRPY(0, p + sum_pitch, 0).getY();
	pose.pose.orientation.z = tf::createQuaternionFromRPY(0, p + sum_pitch, 0).getZ();
	pose.pose.orientation.w = tf::createQuaternionFromRPY(0, p + sum_pitch, 0).getW();
}

void ReceiveCallback(const visualization_msgs::Marker mrk)
{
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

		switch(mrk.id){
		case 7 : rotateOrientation(cam_base_pose, 1.570795);
				 cam_base_pose.pose.position.x = mrk.pose.position.x + base_pose.pose.position.y;
				 cam_base_pose.pose.position.z = mrk.pose.position.z + base_pose.pose.position.x;
				 break;
		case 8 : rotateOrientation(cam_base_pose, 0);
				 cam_base_pose.pose.position.x = mrk.pose.position.x + base_pose.pose.position.x;
				 cam_base_pose.pose.position.z = mrk.pose.position.z + base_pose.pose.position.y;
				 break;
		case 9 : rotateOrientation(cam_base_pose, -1.570795);
				 cam_base_pose.pose.position.x = mrk.pose.position.x - base_pose.pose.position.y;
				 cam_base_pose.pose.position.z = mrk.pose.position.z - base_pose.pose.position.x;
				 break;
		case 10 : rotateOrientation(cam_base_pose, 3.141592);
				 cam_base_pose.pose.position.x = mrk.pose.position.x - base_pose.pose.position.x;
				 cam_base_pose.pose.position.z = mrk.pose.position.z - base_pose.pose.position.y;
				 break;
		default : rotateOrientation(cam_base_pose, 1.570795);
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
}

int main(int argc, char** argv)
{
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
}
