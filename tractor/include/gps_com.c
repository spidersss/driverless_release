#include"gps_com.h"


int gps_find_head(char* buf_gps, int cur, int len_gps){
	if(buf_gps[cur] == 0x14 && buf_gps[cur+1] == 0x64 && cur + 103 < len_gps)
		return 1;
	else 
		return 0;
}

void gps_get_mes(char* buf_gps, int cur, struct GPS_MESSAGES* gps){
	char lat_[8], lon_[8], height_[8], vel_n_[4], vel_e_[4], vel_d_[4], a_X_[4], a_Y_[4], a_Z_[4], gForce_[4], roll_[4], pitch_[4], head_[4], angular_X_[4], angular_Y_[4], angular_Z_[4];
	int j = cur + 15 + 1;//altitude 
	int k = 0;
	for(; k < 8; k+=1){
		lat_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 8; k+=1){
		lon_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 8; k+=1){
		height_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		vel_n_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		vel_e_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		vel_d_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		a_X_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		a_Y_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		a_Z_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		gForce_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		roll_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		pitch_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		head_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		angular_X_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		angular_Y_[k] = buf_gps[j];
		j += 1;
	}
	k = 0;
	for(; k < 4; k+=1){
		angular_Z_[k] = buf_gps[j];
		j += 1;
	}
	gps->lat = *((double*)lat_) * 180 / pi;
	gps->lon = *((double*)lon_) * 180 / pi;
	gps->height = *((double*)height_);
	gps->vel_n = *((float*)vel_n_);
	gps->vel_e = *((float*)vel_e_);
	gps->vel_d = *((float*)vel_d_);
	gps->a_X = *((float*)a_X_);
	gps->a_Y = *((float*)a_Y_);
	gps->a_Z = *((float*)a_Z_);
	gps->gForce = *((float*)gForce_);
	gps->roll = *((float*)roll_) * 180 / pi;
	gps->pitch = *((float*)pitch_) * 180 / pi;
	gps->head = *((float*)head_) * 180 / pi;
	gps->angular_X = *((float*)angular_X_) * 180 / pi;
	gps->angular_Y = *((float*)angular_Y_) * 180 / pi;
	gps->angular_Z = *((float*)angular_Z_) * 180 / pi;
}

void gps_com(char* buf_gps, int len_gps, struct GPS_MESSAGES* gps){
	int i = 0;
	for(i = 0; i < len_gps; i++){
		if(gps_find_head(buf_gps, i, len_gps)){//find the head 
			gps_get_mes(buf_gps, i, gps);
		}
	}
}

void gps_init(char* buf_gps, int len_gps, struct GPS_MESSAGES* gps){
	int i = 0;
	for(i = 0; i < len_gps; i++){
		if(gps_find_head(buf_gps, i, len_gps)){//find the head 
			gps_get_mes(buf_gps, i, gps);
		}
	}
}

int gps_check(struct GPS_MESSAGES* gps){
	if((abs(gps->lat) < 100 && abs(gps->lon) < 200) && ((abs(gps->vel_n)<100)&&(abs(gps->vel_e)<100)&&(abs(gps->head)<400)) && gps->head < 360)
		return 1;
	else
		return 0;
}
