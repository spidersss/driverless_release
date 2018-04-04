//为消除读取数据的延迟影响，将订阅队列长度定为1
#include "gps_anpp/utilities.h"
using namespace utilities;
driverless_test::control_data control_data;
serial_node::serial_send sendmsg;
double gpsYawCorrector(double gpsYaw);
double tarYawCreator(double endlat, double endlon, double gpslat, double gpslon);
double distanceCal(double beglat, double beglon, double endlat, double endlon, double gpslat, double gpslon);
double distance(double a1, double b1, double a2, double b2);
class gpsHandler
{
public:
	gpsHandler()
	{
		gps_sub = n.subscribe("gps_anpp", 1, &gpsHandler::controlCallback, this);
	  	yaw_pub = n.advertise<std_msgs::Float64>("gps_steer", 1);
		count = 0;
		t_yaw = 0.0;
	    enable_flag = 0;
	    disToend = 3.0;
	}
	void gpsCallback(const driverless_test::gps_data::ConstPtr& gps_msg)
	{
	  
	  ROS_INFO("%dendlat:%f\tendlon:%f\t", count, endlat[count], endlon[count]);m
	  t_yaw = tarYawCreator(endlat[count], endlon[count], gps_msg->lat, gps_msg->lon);
	  yaw_now = gpsYawCorrector(gps_msg->yaw);
	  yaw_error = t_yaw - yaw_now;
	  if(yaw_error < 2 && yaw_error > -2) pid_reset_integral(pid_ctrl);
	  
	  control_data.steer = yaw_error*(-5.0) + 90;
	  ROS_INFO("yaw_error:%f\tdis_error:%f", yaw_error, dis_error);
	  if(control_data.steer < 60) control_data.steer = 60;
	  if(control_data.steer > 120) control_data.steer = 120;
	  ROS_INFO("t_yaw: %f\tnowYaw%f\tdisToend: %f", t_yaw,yaw_now, disToend);	
	  
	  disToend = distance(gps_msg->lat, gps_msg->lon, endlat[count], endlon[count]);
	  if( disToend < 3 && !enable_flag){
		enable_flag = 1;
	  }
	  if(enable_flag){ 
	    if(fabs(gps_msg->yaw - endturn[count]) < 5) {
	      control_data.steer = 90;
		  enable_flag = 0;
	  	  count ++;
	    }
	    else{
	  	  if(endturn[count] > 0) control_data.steer = 60;
		  else control_data.steer = 120;
	    }
	  }
	  yaw_pub.publish(control_data);
	}
protected:
	ros::NodeHandle n;
	ros::Subscriber gps_sub;
	ros::Publisher steer_pub;
    int speed;
    double yaw_error;
    double dis_error;
    double disToend;
    double yaw_now;	
    int count;
    float t_yaw;
	int enable_flag;
    
};
int main(int argc, char **argv)
{

  ros::init(argc, argv, "gps_steer");
  gpsHandler handler;
  ros::spin();

  return 0;
}
double gpsYawCorrector(double gpsYaw)
{
	//if(gpsYaw > 270) return gpsYaw - 360;
	//else return gpsYaw;
	return gpsYaw;
}
double tarYawCreator(double endlat, double endlon, double gpslat, double gpslon)
{
	double t_yaw = atan2((cos(endlat)*(endlon - gpslon)),(endlat - gpslat))*180/pi;
	if(t_yaw < 0) t_yaw = t_yaw + 360;
	//if(t_yaw > 90 ) t_yaw = 180 - t_yaw;
	return t_yaw;
}
double distanceCal(double beglat, double beglon, double endlat, double endlon, double gpslat, double gpslon)
{
	double func_k = (endlat -beglat) / (endlon- beglon);//直线的斜率：y:lat x:lon
	double func_b = endlat - func_k * endlon;
	return (func_k * gpslon - gpslat + func_b) /(sqrt((func_k * func_k)+1))*100000;
}
double distance(double a1, double b1, double a2, double b2)
{
	double t = ((a1-a2)*100000);
	return sqrt(pow((a1-a2)*100000, 2) + pow((b1-b2)*100000,2));
}
