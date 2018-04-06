#include "gps_anpp/record.h"

std::ofstream fout;
class gps_recordHandler
{
public:
	gps_recordHandler()
	{
		fout.open("/home/wuconglei/gps_data/rawdata_record");
		gps_sub = nh.subscribe("gps_anpp", 10, &gps_recordHandler::gps_recordCallback, this);
		
	}
	void gps_recordCallback(const gps_anpp::gps_data::ConstPtr& gps_data)
	{
		fout<<std::setprecision(8)<<std::setiosflags(std::ios::fixed)<<gps_data->lat<<"\t"<<gps_data->lon<<"\n";
	}
protected:
	ros::NodeHandle nh;
	ros::Subscriber gps_sub;
	
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_record");
  gps_recordHandler handler;
  ros::spin();
  fout.close();//位置未定
  return 0;
}
