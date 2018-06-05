#ifndef _WRITE2FILES_H_
#define _WRITE2FILES_H_

#include"serial_open.h"

void write2file(struct GPS_MESSAGES* gps_msg, struct CTRL_MESSAGES* ctrl_msg, FILE* f);

#endif
