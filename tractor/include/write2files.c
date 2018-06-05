#include"write2files.h"

void write2file(struct GPS_MESSAGES* gps_msg, struct CTRL_MESSAGES* ctrl_msg, FILE* f){
	int duphandle;
	fprintf(f,"%.8lf\t%.8lf\n", gps_msg->lat, gps_msg->lon);
	fprintf(f,"%d\t%d\t%d\n", ctrl_msg->speed, ctrl_msg->steer, ctrl_msg->power);
	fflush(f);
	duphandle = dup(fileno(f));
	close(duphandle);			
}
