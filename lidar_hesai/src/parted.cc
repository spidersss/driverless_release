#include "lidar_hesai/traffic_cone.h"
ros::Publisher pubxyz;
void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_init;
	pcl::fromROSMsg(cloud_msg, cloud_init);
	
	PointCloud cloud_parted;
	cloud_parted = space_part(cloud_init, 3.0, -20.0, 0.2);
	
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(cloud_parted, output);
	pubxyz.publish(output);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "pcl_test");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("pandar_points", 10, cloud_cb);
	pubxyz = n.advertise<sensor_msgs::PointCloud2> ("parted_points", 10);
	ros::spin();
}
