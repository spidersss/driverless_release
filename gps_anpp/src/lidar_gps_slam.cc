#include "gps_anpp/lidar_gps_slam.h"
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#define PI 3.1415927
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;  
struct POS
{
	double x;
	double y;
};
std::vector<POS> slam_pos;
double lon_x;
double lat_y;
int count;
ros::Time current_time, last_time;
nav_msgs::Path path;
class slam
{
public:
	slam()
	{
		sublidar = n.subscribe("cluster_points", 10,  &slam::cloud_cb, this);
		subgps = n.subscribe("gps_anpp", 10,  &slam::gps_cb, this);
		pubpath = n.advertise<nav_msgs::Path> ("trajectory", 10);
		pubslam = n.advertise<sensor_msgs::PointCloud2> ("trajectory", 10);
		count = 0;
	}
	void gps_cb(const gps_anpp::gps_data::ConstPtr& gps_msg)
	{
		if(count < 5){//头5次确定初始点经纬度lon_x,lat_y;
			lon_x += gps_msg->lon;
			lat_y += gps_msg->lat;
			count++;
		}
		else if(count == 5){
			lon_x /= 5;
			lat_y /= 5;
			count++;
		}
		else{
			current_time = ros::Time::now();
	   		last_time = ros::Time::now();

			path.header.stamp=current_time;
			path.header.frame_id="odom";
			
			x0 = cos(gps_msg->lat/180.0*PI)*(gps_msg->lon - lon_x)*111000;
			y0 = (gps->lat - lat_y)*111000;
			yaw = gps_msg->yaw/180.0*PI;
			
		    current_time = ros::Time::now();
		    geometry_msgs::PoseStamped this_pose_stamped;
		    this_pose_stamped.pose.position.x = x0;
		    this_pose_stamped.pose.position.y = y0;

		    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(yaw);
		    this_pose_stamped.pose.orientation.x = goal_quat.x;
		    this_pose_stamped.pose.orientation.y = goal_quat.y;
		    this_pose_stamped.pose.orientation.z = goal_quat.z;
		    this_pose_stamped.pose.orientation.w = goal_quat.w;

		    this_pose_stamped.header.stamp=current_time;
		    this_pose_stamped.header.frame_id="odom";
		    path.poses.push_back(this_pose_stamped);
		    path_pub.publish(path);
		    last_time = current_time;
        }
	}
	void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		if(count > 5){
		PointCloud cloud;
		pcl::fromROSMsg(cloud_msg, cloud);
		std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
		for(it = cloud.points.begin(); it != cloud.points.end(); it++)
		{	
		
			POS pos;
			double x = it->x;
			double y = it->y;
			pos.x = sqrt(x*x + y*y) * sin(yaw) + x0;
			pos.y = sqrt(x*x + y*y) * cos(yaw) + y0;
			int flag = 0;
			for(int i = slam_pos.size-1; i >=0; i--){
				if(fabs(slam_pos[i].x-pos.x)<1.0 && fabs(slam_pos[i].y - pos.y)<1.0){
					slam_pos[i].x = (slam_pos[i].x + pos.x)/2;
					slam_pos[i].y = (slam_pos[i].y + pos.y)/2;
					flag = 1;
					break;
				}
			}
			if(flag == 0) slam_pos.push_back(pos);
		}
		
		PointCloud cloud_output;
		pcl::PointXYZ pointxyz;
		for(int i = 0; i < slam_pos.size(); i++){
			pointxyz.x = slam_pos[i].x;
			pointxyz.y = slam_pos[i].y;
			cloud_output.push_back(pointxyz);		
		}
		cloud_output.header.frame_id = "odom";
 		cloud_output.width = cloud_center.points.size ();
		cloud_output.height = 1;
		cloud_output.is_dense = false;
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_output, output);
		pubpath.publish(output);
		}
	}
	
	
protected:
	ros::NodeHandle n;
	ros::Subscriber sublidar;
	ros::Subscriber subgps;
	ros::Publisher pubpath;
	double x0;
	double y0;
	float yaw;
	
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "slam");
	slam handler;
	ros::spin();
	return 0;
}
