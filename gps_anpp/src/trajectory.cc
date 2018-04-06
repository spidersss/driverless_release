#include "gps_anpp/trajectory.h"
using namespace trajectory;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_trajectory");//不确定是否保留
	fin.open("/home/wuconglei/gps_data/rawdata_record");
	while(fin>>latitude>>longitude)
	{
		std::cout<<std::setprecision(8)<<std::setiosflags(std::ios::fixed)<<latitude<<"\t"<<longitude<<"\n";
		lat.push_back(latitude);
		lon.push_back(longitude);
	}
	fin.close();
	fout.open("/home/wuconglei/gps_data/trajectory_points");
	last_lat = lat[0];
	last_lon = lon[0];
	fout<<std::setprecision(8)<<std::setiosflags(std::ios::fixed)<<last_lat<<"\t"<<last_lon<<"\n";
	for(int i = 1; i < lat.size(); i++)
	{	
		distance = sqrt(pow((last_lat - lat[i]), 2) + pow((last_lon - lon[i])*cos(lat[i]), 2)) * 100000;
		//车速10m/s, 两种方式判断没有跳点：1、下一个经纬度在一定范围内变化; 2、经纬度变化到一定程度需要必要的时间,用变量count记录两点间中间的间隔，如果太小说明是跳变;
		if(distance > 1.0){
			if(distance < 2.0) {
				fout<<std::setprecision(8)<<std::setiosflags(std::ios::fixed)<<lat[i]<<"\t"<<lon[i]<<"\n";
				last_lat = lat[i];
				last_lon = lon[i];
			}
			else{
				last_lat = lat[i+1];
				last_lon = lon[i+1];
				fout<<std::setprecision(8)<<std::setiosflags(std::ios::fixed)<<last_lat<<"\t"<<last_lon<<"\n";
				i++;
			} 		
		}
		
	}
	fout.close();
	return 0;
}


