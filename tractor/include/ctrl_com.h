#ifndef _CTRL_COM_H_
#define _CTRL_COM_H_

#include"serial_open.h"

struct CTRL_MESSAGES{
	unsigned char speed;
	unsigned char steer;
	unsigned char power;
};

void ctrl_send(unsigned char se_buf[], struct CTRL_MESSAGES* send, int fd_ctrl, int len);

#endif
