#include"./include/serial_open.h"
#include"./include/gps_com.h"
#include"./include/navigation.h"
#include"./include/ctrl_com.h"
#include"./include/write2files.h"

int main(int argc, char **argv)
{
    int fd_gps = 0, fd_ctrl = 0;
	fd_gps = dev_open("/dev/ttyS0");
	fd_ctrl = dev_open("/dev/ttyS1");

	struct GPS_MESSAGES* gps_msg;
	struct CTRL_MESSAGES* ctrl_msg;
	
	double endlat[3], endlon[3], endturn[3];
	endlat[0] = 31.88668392, endlon[0] = 118.80989973, endturn[0] = 90;//0为起点，转角不起作用
  	endlat[1] = 31.88664502, endlon[1] = 118.80960257, endturn[1] = 180.0;
 	endlat[2] = 33.88722013, endlon[2] = 120.81050525, endturn[2] = 270.0;
	
	char buf_gps[READ_NUM];
	int len_gps = 0;

	unsigned char se_buf[1];
	FILE* f = fopen("record.txt","a");	
	int count = 1;
	while (1)
	{
		len_gps = read(fd_gps, buf_gps, READ_NUM);
		gps_init(buf_gps, len_gps, gps_msg);
		gps_com(buf_gps, len_gps, gps_msg);
		printf("%f\t%f\t%f\n", gps_msg->lat, gps_msg->lon,gps_msg->head);

		if(!gps_check(gps_msg)) continue;			
		if(gps_check(gps_msg)){
			(*ctrl_msg) = navigation(gps_msg, endlat, endlon, endturn, &count);
			ctrl_send(se_buf, ctrl_msg, fd_ctrl, 1);
			//write2file(gps_msg, ctrl_msg, f);
		}
		usleep(40000);
	}
	close(fd_gps);
	close(fd_ctrl);  
	fclose(f);

	return 0;

}
