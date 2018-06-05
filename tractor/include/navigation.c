#include "navigation.h"

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

struct CTRL_MESSAGES navigation(struct GPS_MESSAGES* gps_msg, double endlat[], double endlon[], double endturn[], int* count)
{
  double yaw_error = 0, dis_error = 0, disToend= 3, yaw_now = 0;	
  int enable_flag = 0;
  float t_yaw = 0.0;
  
  struct CTRL_MESSAGES ctrl_msg;
  ctrl_msg.speed = 5;
  ctrl_msg.steer = 0;
  ctrl_msg.power = 1;

  time_t rawtime;  
  struct tm * ptm;  
  time(&rawtime);  
  ptm = gmtime(&rawtime);  
  printf ("Beijing (China) :     %2d:%02d:%02d\n", (ptm->tm_hour+CCT)%24, ptm->tm_min, ptm->tm_sec);  
  printf("count:%d\tendlat:%f\tendlon:%f\n", *count, endlat[*count], endlon[*count]);
  t_yaw = tarYawCreator(endlat[*count], endlon[*count], gps_msg->lat, gps_msg->lon);
  yaw_now = gpsYawCorrector(gps_msg->head);
  yaw_error = t_yaw - yaw_now;
	  
  ctrl_msg.steer = yaw_error*(-5.0) + 90;
  dis_error = distanceCal(endlat[*count-1], endlon[*count-1], endlat[*count], endlon[*count], gps_msg->lat, gps_msg->lon);
  disToend = distance(gps_msg->lat, gps_msg->lon, endlat[*count], endlon[*count]);
  printf("t_yaw: %f\tnowYaw%f\tdisToend: %f\n", t_yaw,yaw_now, disToend);	
  printf("yaw_error:%f\tdis_error:%f\n", yaw_error, dis_error);
  if(ctrl_msg.steer < 60) ctrl_msg.steer = 60;
  if(ctrl_msg.steer > 120) ctrl_msg.steer = 120;
	  
  if( disToend < 3 && !enable_flag){
		enable_flag = 1;
  }
  if(enable_flag){ 
	  if(fabs(gps_msg->head - endturn[*count]) < 5) {
	    ctrl_msg.steer = 90;
		enable_flag = 0;
	  	*count ++;
	  }
	  else{
	  	if(endturn[*count] > 0) ctrl_msg.steer = 60;
		else ctrl_msg.steer = 120;
      }
  }
  return ctrl_msg;
}
