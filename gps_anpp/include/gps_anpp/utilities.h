#ifndef __UTILITIES_H_
#define __UTILITIES_H_
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include "gps_anpp/gps_data.h"
#include <vector>
#include <fstream>
#ifndef M_PI
#define M_PI 3.1415926
#endif

double gpsYawCorrector(double gpsYaw);
double tarYawCreator(double endlat, double endlon, double gpslat, double gpslon);
double distanceCal(double beglat, double beglon, double endlat, double endlon, double gpslat, double gpslon);
double distance(double a1, double b1, double a2, double b2);

namespace utilities{
    double yaw_error = 0.0;
    double dis_error = 0.0;
    double disToend = 3.0;
    double yaw_now = 0.0;	
    int count = 0;
    float t_yaw = 0.0;
	double latitude = 0.0;
	double longitude = 0.0;
}
#endif

double gpsYawCorrector(double gpsYaw)
{
	//if(gpsYaw > 270) return gpsYaw - 360;
	//else return gpsYaw;
	return gpsYaw;
}
double tarYawCreator(double endlat, double endlon, double gpslat, double gpslon)
{
	double t_yaw = atan2((cos(endlat)*(endlon - gpslon)),(endlat - gpslat))*180/M_PI;
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
