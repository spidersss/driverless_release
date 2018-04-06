#include "gps_anpp/utilities.h"
using namespace utilities;

ros::Subscriber gps_sub;
ros::Publisher steer_pub;
std_msgs::Float64 steer;
std::vector<double> endlat;
std::vector<double> endlon;
void gpsCallback(const gps_anpp::gps_data::ConstPtr& gps_msg)
{
	
	ROS_INFO("count:%d\tendlat:%f\tendlon:%f\t", count, endlat[count], endlon[count]);
	t_yaw = tarYawCreator(endlat[count], endlon[count], gps_msg->lat, gps_msg->lon);
	yaw_now = gpsYawCorrector(gps_msg->yaw);
	yaw_error = t_yaw - yaw_now;
	  
	steer.data = yaw_error*(-5.0) + 90;
	ROS_INFO("yaw_error:%f\tdis_error:%f", yaw_error, dis_error);
	if(steer.data < 60) steer.data = 60;
	if(steer.data > 120) steer.data = 120;
	ROS_INFO("t_yaw: %f\tnowYaw%f\tdisToend: %f", t_yaw,yaw_now, disToend);	
	  
	disToend = distance(gps_msg->lat, gps_msg->lon, endlat[count], endlon[count]);
	if( disToend < 3 ){
		if(count < endlat.size() - 1) count ++;
	}
	steer_pub.publish(steer);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gps_steer");
  ros::NodeHandle n;
  std::ifstream fin("/home/wuconglei/gps_data/trajectory_points");
  while(fin>>latitude>>longitude){
  	std::cout<<std::setprecision(8)<<std::setiosflags(std::ios::fixed)<<latitude<<"\t"<<longitude<<"\n";
  	endlat.push_back(latitude);
  	endlon.push_back(longitude);
  }
  steer_pub = n.advertise<std_msgs::Float64>("gps_steer", 1);
  //为消除读取数据的延迟影响，将订阅队列长度定为1
  gps_sub = n.subscribe("gps_anpp", 1, gpsCallback);
  ros::spin();

  return 0;
}
