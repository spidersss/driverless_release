#include "gps_anpp/gps_anpp.h"
using namespace gps_anpp;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_anpp");
  
  //fd_gps = dev_open("/dev/ttyS0");
  if(argc != 2) {
  	std::cout<<"Error: please add the address of the device!"<<std::endl;
  	return 1;
  }
  else fd_gps = dev_open(argv[1]);

  ros::NodeHandle n;
  ros::Publisher gps_pub = n.advertise<gps_anpp::gps_data>("gps_anpp", 1);
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
  	gps_anpp::gps_data gps_data;
    len_gps = read(fd_gps, buf_gps, READ_NUM);
    gps_com(buf_gps, len_gps, &gps_mes);
    if(!gps_check(&gps_mes)) continue;			
    gps_data.id = count;
    gps_data.len = 3;
    gps_data.lat = gps_mes.lat;
    gps_data.lon = gps_mes.lon;
    gps_data.yaw = gps_mes.yaw;

    //ROS_INFO("id: %d\tlen: %d\tlat: %f\tlon: %f\tyaw: %f", gps_data.id,gps_data.len,gps_data.lat,gps_data.lon,gps_data.yaw);
	//printf("%.8lf\t%.8lf\t%.1lf\n", gps_data.lat, gps_data.lon, gps_data.yaw);
	printf("%.8lf,%.8lf\n", gps_data.lat, gps_data.lon);
   	gps_pub.publish(gps_data);
   	
    ros::spinOnce();

    loop_rate.sleep();
    
    ++count;
  }


  return 0;
}

