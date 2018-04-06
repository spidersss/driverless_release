#ifndef __TRAJECTORY_H_
#define __TRAJECTORY_H_

#include "ros/ros.h"
#include "sstream"
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

namespace trajectory{
	std::ifstream fin;
	std::ofstream fout;
	double latitude;
	double longitude;
	std::vector<double> lat;
	std::vector<double> lon;
	double last_lat;
	double last_lon;
	double distance;
}

#endif
