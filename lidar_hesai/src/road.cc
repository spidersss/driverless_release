#include "lidar_hesai/traffic_cone.h"
ros::Publisher pubxyz;
ros::Publisher pub_steer;
class cloudHandler
{
public:
	cloudHandler()
	{
		pcl_sub = nh.subscribe("rslidar_points", 10, &cloudHandler::cloudCB, this);
		pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_partitioned", 1);
	}
	
	void cloudCB(const sensor_msgs::PointCloud2 &input)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;
		sensor_msgs::PointCloud2 output;
		
		pcl::fromROSMsg(input , cloud);
		
		float resolution = 128.0f;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
		
		octree.setInputCloud(cloud.makeShared());
		octree.addPointsFromInputCloud();
		
		pcl::PointXYZ center_point;
		center_point.x = -2.6;
		center_point.y = 2.3;
		center_point.z = -0.3;
		
		float radius = 1.0;
		std::vector<int> radiusIdx;
		std::vector<float> radiusSQDist;
		if(octree.radiusSearch(center_point, radius, radiusIdx, radiusSQDist) > 0)
		{
			for(size_t i = 0; i < radiusIdx.size(); i++)
			{
				cloud_partitioned.points.push_back(cloud.points[radiusIdx[i]]);
			}
		}
		
		pcl::toROSMsg(cloud_partitioned, output);
		output.header.frame_id = "odom";
		pcl_pub.publish(output);
	}

protected:
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub;
	ros::Publisher pcl_pub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_partitioned");
	cloudHandler handler;
	ros::spin();
	return 0;
}
void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_init;
	pcl::fromROSMsg(cloud_msg, cloud_init);
	
	PointCloud cloud_parted;
	cloud_parted = space_part(cloud_init, 3.0, -20.0, 0);
	
	//PointCloud cloud_filtered;
	//cloud_filtered = outlier_filter(cloud_parted, 5, 1.0);//滤去离群值，参数未调好
	PointCloud cloud_center;
	cloud_center = center_cluster(cloud_parted, 0.2, 5, 2500);
	
	std_msgs::Float64 steer;
	steer.data = steerCreator(cloud_center);
	pub_steer.publish(steer);
	
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
