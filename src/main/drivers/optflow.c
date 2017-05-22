#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "common/maths.h"
#include <platform.h>
#include "build/debug.h"
#include "fc/fc_tasks.h"
#include "config/parameter_group.h"
#include "drivers/dma.h"
#include "drivers/nrf2401.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "common/crc.h"
#include "optflow.h"
#include "rx/rx.h"

struct flow_data flow;

static void flowDataReceive(uint16_t data);
static void flow_stab(void);

void flow_init(void)
{
	openSerialPort(SERIAL_PORT_UART2,FUNCTION_TELEMETRY_MAVLINK, flowDataReceive, 57600, MODE_RX, SERIAL_NOT_INVERTED);//SERIAL_STOPBITS_1
	stab.cmd[0] = 0;
	stab.cmd[1] = 0;
	stab.error_vx_int = 0;
	stab.error_vy_int = 0;
}


uint8_t flow_buf[30],flow_state[4];
void flowDataReceive(uint16_t data)
{
	static uint8_t count, i, buffer[5];

	switch(count)
	{
   		case 0: if(data == 0xAA)	
				count = 1;
				break;

 		case 1: if(i < 4)	
					buffer[i++] = data;
				else {buffer[i] = data;i = 0;count = 2;}
				break;
		
		case 2:if(buffer[4] == ((buffer[0]^buffer[1]^buffer[2]^buffer[3])&0xFF)) {
					memcpy(&flow.x,&buffer[0],2);
					memcpy(&flow.y,&buffer[2],2);
				}
				count = 0;
				break;

		default:count = 0;i = 0;break;
	}
	
	flow.comp_x = bound(flow.x,200,-200);	
	flow.comp_y = bound(flow.y,200,-200);
	debug[0] = flow.x;
	debug[1] = flow.y;
	


/*
	static uint8_t check = 0,data_cnt = 0;
	uint8_t temp[4];
	float *p = (float*)temp;
	bool Catch_flag = false;
	switch(check)
	{
   		case 0: if(data == 0xFE)check = 1;break;
 		case 1: if(data == 0x1A)check = 2;
					else check = 0;
				break;
		case 2:	if(data_cnt < 4){
					check = 2;
					flow_state[data_cnt++] = data;
				}else{
					check = 3;
					data_cnt = 0;
					flow_buf[data_cnt++] = data;
				}
				break;
		case 3:	if(flow_state[3] == 100){
					if(data_cnt < 26){
						check = 3;
						flow_buf[data_cnt++] = data;
					}else{
						check=4;
						data_cnt=0;
					}
				}else{
					check = 0;
					data_cnt = 0;
				}
				break;
		case 4: check = 0;data_cnt = 0;Catch_flag = true;break;
        default:check = 0;data_cnt = 0;break;
	}
                
	if(Catch_flag){
		//flow.time_sec=(flow_buf[7]<<64)|(flow_buf[6]<<56)|(flow_buf[5]<<48)|(flow_buf[4]<<40)|(flow_buf[3]<<32)|(flow_buf[2]<<16)|(flow_buf[1]<<8)|(flow_buf[0]);
		//flow.quality = flow_buf[25];//Optical flow quality / confidence. 0: bad, 255: maximum quality
		temp[0] = flow_buf[8];
		temp[1] = flow_buf[9];
		temp[2] = flow_buf[10];
		temp[3] = flow_buf[11];
		flow.comp_x = 100 * (*p);//unit:cm/s +-60

		temp[0] = flow_buf[12];
		temp[1] = flow_buf[13];
		temp[2] = flow_buf[14];
		temp[3] = flow_buf[15];
		flow.comp_y = 100 * (*p);

		temp[0] = flow_buf[16];
		temp[1] = flow_buf[17];
		temp[2] = flow_buf[18];
		temp[3] = flow_buf[19];
		flow.height = *p;//unit:m
		flow.x = (int16_t)(flow_buf[20] + (flow_buf[21] << 8));//+-100
		flow.y = (int16_t)(flow_buf[22] + (flow_buf[23] << 8));
	}
	//debug[0] = flow.height;
	//debug[0] = flow.x;
	//debug[2] = flow.y;
	//debug[1] = flow.comp_x;
	//debug[2] = flow.comp_y;
*/
}


#ifdef PX4FLOW

#define rol 0
#define pit 1

struct flow_stab stab;
struct angle angle_flow;

void taskOptflow(void)
{	
	if((rcData[4] > 1800) \
	 && (rcData[0] > 1450) && (rcData[0] < 1550) && (rcData[1] > 1450) && (rcData[1] < 1550))
		flow_stab();
	else {
		stab.cmd[rol] = 0;
		stab.cmd[pit] = 0;
		stab.error_vx_int = 0;
		stab.error_vy_int = 0;
		stab.error_x = 0;
		stab.error_y = 0;
	}

}
#endif


//				 roll  pitch
float KP1[2] = {0.6, 0.6};
float KP2[2] = {1.8, 1.8};
float KI[2]  = {1.0, 1.1};
float KD[2]  = {1.6, 1.6};
/*
float KP2[2] = {1.8, 1.8};
float KI[2]  = {0.1, 0.1};
float KD[2]  = {0.6, 0.6};
*/

void flow_stab(void)
{
			static float old_error_vx,old_error_vy;
			if((flow.comp_x < 6.0) && (flow.comp_x > -6.0))
				flow.comp_x = 0;
			if((flow.comp_y < 6.0) && (flow.comp_y > -6.0))
				flow.comp_y = 0;

			float error_vx = constrainf(flow.comp_x,-40,40);
			float error_vy = constrainf(-flow.comp_y,-40,40);
			//float error_vx = -flow.x * flow.height/2;//cm/s
			//float error_vy = flow.y * flow.height/2;
			//debug[0] = error_vx;//+-60

			stab.error_vx_int += error_vx / 50;
			stab.error_vx_int = constrainf(stab.error_vx_int,-40,40);
			stab.error_vy_int += error_vy / 50;
			stab.error_vy_int = constrainf(stab.error_vy_int,-40,40);
			//debug[0] = stab.error_vx_int;//+-30

			stab.cmd[rol] = constrainf((KP2[rol] * error_vx),-100,100) + \
							constrainf((KI[rol] * stab.error_vx_int),-40,40) + \
							constrainf((KD[rol] * (error_vx - old_error_vx)),-40,40);
			stab.cmd[pit] = constrainf((KP2[pit] * error_vy),-100,100) + \
							constrainf((KI[pit] * stab.error_vy_int),-40,40) + \
							constrainf((KD[pit] * (error_vy - old_error_vy)),-40,40);
			stab.cmd[rol] = constrainf(stab.cmd[rol],-150,150);
			stab.cmd[pit] = constrainf(stab.cmd[pit],-150,150);

			debug[2] = stab.cmd[rol];
			debug[3] = stab.cmd[pit];
			//debug[1] = error_vx - old_error_vx;//+-15
			old_error_vx = error_vx;
			old_error_vy = error_vy;


/*
			static float old_error_vx,old_error_vy;			
			if((flow.comp_x < 6.0) && (flow.comp_x > -6.0))
				flow.comp_x = 0;
			if((flow.comp_y < 6.0) && (flow.comp_y > -6.0))
				flow.comp_y = 0;
			//debug[0] = flow.comp_x;
			

			stab.error_x += flow.comp_x / 50;
			stab.error_y -= flow.comp_y / 50;	
			stab.error_x = constrainf(stab.error_x,-50,50);
			stab.error_y = constrainf(stab.error_y,-50,50);
			//debug[1] = stab.error_x;//5 < 50 

			float error_vx = -stab.error_x * KP1[rol] - flow.comp_x;
			float error_vy = -stab.error_y * KP1[pit] - flow.comp_y;
			error_vx = constrainf(error_vx,-60,60);
			error_vy = constrainf(error_vy,-60,60);
			//debug[1] = error_vx;//30-50

			stab.error_vx_int += error_vx / 50;
			stab.error_vy_int += error_vy / 50;
			stab.error_vx_int = constrainf(stab.error_vx_int,-400,400);
			stab.error_vy_int = constrainf(stab.error_vy_int,-400,400);
			//debug[2] = stab.error_vx_int;//100-200

			stab.cmd[rol] = constrainf((KP2[rol] * error_vx),-120,120) +  \
							constrainf((KI[rol] * stab.error_vx_int),-40,40) + \
							constrainf((KD[rol] * (error_vx - old_error_vx)),-50,50);
			stab.cmd[pit] = constrainf((KP2[pit] * error_vy),-150,150) + \
							constrainf((KI[pit] * stab.error_vy_int),-40,40) + \
							constrainf((KD[pit] * (error_vy - old_error_vy)),-50,50);

			stab.cmd[rol] = constrainf(stab.cmd[rol],-150,150);
			stab.cmd[pit] = constrainf(stab.cmd[pit],-150,150);
			debug[2] = stab.cmd[rol];
			debug[3] = stab.cmd[pit];
			//debug[3] = error_vx - old_error_vx;//80
			old_error_vx = error_vx;
			old_error_vy = error_vy;
*/
}   
                
