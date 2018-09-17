#include "gps_anpp/lidar_gps_slam.h"
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
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
	void gps_cb()
	{
		if(count < 5){//头5次确定初始点经纬度lon_x,lat_y;
			lon_x += gps_msg.lat;
			 
		}
	}
	void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		if(count == 5){
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
		cloud_output.header.frame_id = "pandar";
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
