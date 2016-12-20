#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "common/maths.h"
#include <platform.h>
#include "build/debug.h"
#include "fc/fc_tasks.h"
#include "config/parameter_group.h"
#include "drivers/dma.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "common/crc.h"
#include "optflow.h"
#include "rx/rx.h"
#ifdef NRF
#include "nrf2401.h"
#endif

struct flow_data flow;

static void flowDataReceive(uint16_t data);

void flow_init(void)
{
	openSerialPort(SERIAL_PORT_UART2,FUNCTION_TELEMETRY_MAVLINK, flowDataReceive, 115200, MODE_RX, SERIAL_NOT_INVERTED);//SERIAL_STOPBITS_1
	stab.cmd[0] = 0;
	stab.cmd[1] = 0;
	stab.error_vx_int = 0;
	stab.error_vy_int = 0;
}

uint8_t flow_buf[30],flow_state[4];
void flowDataReceive(uint16_t data)
{
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
}


#ifdef PX4FLOW
struct flow_stab stab;
uint8_t PID_Roll[3] = {200,100,80};//X
uint8_t PID_Pitch[3] = {200,100,80};//Y

void taskOptflow(void)
{	
	//AUX1
	if(rcData[4] > 1800){
		if((rcData[0] > 1450) && (rcData[0] < 1550) && (rcData[1] > 1450) && (rcData[1] < 1550)){
			/*
			static float old_error_x,old_error_y;

			stab.error_x -= flow.comp_x / 50;//uint:cm/s
			stab.error_y += flow.comp_y / 50;			

			debug[0] = stab.error_x;
			debug[1] = stab.error_y;		

			stab.error_vx_int += stab.error_x;
			stab.error_vy_int += stab.error_y;
			debug[2] = stab.error_vx_int;
			debug[3] = stab.error_vy_int;
			
			stab.cmd[0] = PID_Roll[0] * stab.error_x +
							PID_Roll[1] * constrainf(stab.error_vx_int, -100,100) +
							PID_Roll[2] * constrainf((stab.error_x - old_error_x),-100,100);
			//debug[2] = error_vx - old_error_vx;//+-100

			stab.cmd[1] = PID_Pitch[0] * stab.error_y +
							PID_Pitch[1] * constrainf(stab.error_vy_int,-100,100) +
							PID_Pitch[2] * constrainf((stab.error_y - old_error_y),-100,100);
			//debug[0] = error_vy - old_error_vy;//
			stab.cmd[0] = constrainf((stab.cmd[0] / 100),-130,130);
			stab.cmd[1] = constrainf((stab.cmd[1] / 100),-130,130);
			old_error_x = stab.error_x;
			old_error_y = stab.error_y;
			*/
			
			static float old_error_vx,old_error_vy;
			float error_vx = -flow.comp_x;//cm/s
			float error_vy = flow.comp_y;

			//float error_vx = -flow.x * flow.height/2;//cm/s
			//float error_vy = flow.y * flow.height/2;
			//debug[0] = error_vx;//+-60

			stab.error_vx_int += error_vx / 50;
			stab.error_vy_int += error_vy / 50;
			//debug[1] = stab.error_vx_int;//+-50

			//PID
			stab.cmd[0] = constrainf((PID_Roll[0] * error_vx),-10000,10000) +
							constrainf((PID_Roll[1] * stab.error_vx_int),-4000,4000) +
							constrainf((PID_Roll[2] * (error_vx - old_error_vx)),-2500,2500);
			//debug[2] = error_vx - old_error_vx;//+-30

			stab.cmd[1] = constrainf((PID_Pitch[0] * error_vy),-10000,10000) +
							constrainf((PID_Pitch[1] * stab.error_vy_int),-4000,4000) +
							constrainf((PID_Pitch[2] * (error_vy - old_error_vy)),-2500,2500);

			stab.cmd[0] = constrainf((stab.cmd[0] / 100),-150,150);
			stab.cmd[1] = constrainf((stab.cmd[1] / 100),-150,150);
			old_error_vx = error_vx;
			old_error_vy = error_vy;

			debug[0] = stab.cmd[0];//roll
			debug[1] = stab.cmd[1];//pitch
		}
		else{
			stab.cmd[0] = 0;
			stab.cmd[1] = 0;
			stab.error_vx_int = 0;
			stab.error_vy_int = 0;

			stab.error_x = 0;
			stab.error_y = 0;
		}
	}
	else{
		stab.cmd[0] = 0;
		stab.cmd[1] = 0;
		stab.error_vx_int = 0;
		stab.error_vy_int = 0;
		
		stab.error_x = 0;
		stab.error_y = 0;
	}
}
#endif

        
                
