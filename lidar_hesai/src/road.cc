#include "lidar_hesai/traffic_cone.h"
#include "std_msgs/Float64MultiArray.h"
/////与parted合并
class cloudHandler
{
public:
	cloudHandler()
	{
		pub_z = n.advertise<std_msgs::Float64MultiArray> ("height_z", 10);
		pubxyz = n.advertise<sensor_msgs::PointCloud2> ("filter_middle", 10);
		sub = n.subscribe("pandar_points", 10, &cloudHandler::cloud_cb, this);
		for(int i = 0; i < 80; i++) heightMax.push_back(-1.0);
		for(int i = 0; i < 80; i++) heightMin.push_back(1.0);
	}
	void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
	{
		PointCloud cloud_init;
		pcl::fromROSMsg(cloud_msg, cloud_init);
	
		PointCloud cloud_parted;
		cloud_parted = space_part(cloud_init, 0.0, -20.0);
	
		std::vector<pcl::PointXYZ, Eigen::aligned_allocator_indirection<pcl::PointXYZ> >::iterator it;
		for(it = cloud_parted.points.begin(); it != cloud_parted.points.end(); it++)
		{	
			int index = int(-1*it->y*4);
			if(it->y >0 ) index = 0;
			if(index > 80) continue;
			std::cout<<index<<std::endl;
			/*
			std::cout<<it->y<<std::endl;
			std::cout<<index<<std::endl;
			std::cout<<"Max:"<<heightMax[index]<<std::endl;
			std::cout<<"Min:"<<heightMin[index]<<std::endl;
			*/
			if(it->z > heightMax[index]) heightMax[index] = it->z;
			if(it->z < heightMin[index]) heightMin[index] = it->z;
		}
		if(heightMax[0]  < -0.5) heightMax[0] =  heightMax[1];
		for(int j = 0; j <20 ; j++){
			if((heightMax[j] - heightMin[j]) > 0.1) heightMax[j] = heightMin[j]+0.05;
			if(heightMax[j] < -0.5 && j != 0) heightMax[j] = heightMax[j-1] + 0.05;
		}
		std_msgs::Float64MultiArray heights;
		heights.data = heightMax;
		pub_z.publish(heights);
	
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(cloud_parted, output);
		pubxyz.publish(output);
	}
	
protected:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub_z;
	ros::Publisher pubxyz;
	std::vector<double> heightMax;
	std::vector<double> heightMin;
};
int main(int argc, char** argv)
{
	ros::init (argc, argv, "road");
	cloudHandler handler;
	ros::spin();
	return 0;
}
