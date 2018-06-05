#include"ctrl_com.h"

void ctrl_send(unsigned char se_buf[], struct CTRL_MESSAGES* send, int fd_ctrl, int len ){
	se_buf[0] = send->steer;
	write(fd_ctrl, se_buf, len);
	printf("speed:%d\tsteer:%d\tpower:%d\n", send->speed, send->steer, send->power);  
}


