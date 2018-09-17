#ifndef __LIDAR_GPS_SLAM_H_
#define	__LIDAR_GPS_SLAM_H_

#include "ros/ros.h"
#include <sstream>
#include "gps_anpp/input.h"
#include "gps_anpp/rawdata.h"
#include "gps_anpp/gps_data.h"
#include "gps_anpp/gps_anpp.h"
#define BUF_LENGTH 256
namespace gps_anpp{
	int fd_gps;
	int count = 0;
	struct GPS_MESSAGES gps_mes;	
  	char buf_gps[BUF_LENGTH];
  	int len_gps = 0;
}
#endif
