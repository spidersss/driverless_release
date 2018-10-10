#include "lidar_hesai/traffic_cone_b.h"
ros::Publisher pubxyz;
ros::Publisher pub_steer;
//////分开运行，多线程
void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_init;
	pcl::fromROSMsg(cloud_msg, cloud_init);
	
	PointCloud cloud_parted;
	cloud_parted = space_part(cloud_init, 3.0, -20.0, 0);
	
	//PointCloud cloud_filtered;
	//cloud_filtered = outlier_filter(cloud_parted, 5, 1.0);//滤去离群值，参数未调好
	//PointCloud cloud_center;
	//cloud_center = center_cluster(cloud_parted, 0.2, 5, 2500);
	
	//std_msgs::Float64 steer;
	//steer.data = steerCreator(cloud_center);
	//pub_steer.publish(steer);
	
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(cloud_parted, output);
	pubxyz.publish(output);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "pcl_test");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("pandar_points", 10, cloud_cb);
	pubxyz = n.advertise<sensor_msgs::PointCloud2> ("filter_z", 10);
	pub_steer = n.advertise<std_msgs::Float64> ("lidar_steer", 10);
	ros::spin();
}
