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
		flow.comp_x = 100 * (*p);//单位：cm

		temp[0] = flow_buf[12];
		temp[1] = flow_buf[13];
		temp[2] = flow_buf[14];
		temp[3] = flow_buf[15];
		flow.comp_y = 100 * (*p);

		temp[0] = flow_buf[16];
		temp[1] = flow_buf[17];
		temp[2] = flow_buf[18];
		temp[3] = flow_buf[19];
		flow.height = 100 * (*p);
		flow.vx = (int16_t)(flow_buf[20] + (flow_buf[21] << 8));//20 50 80
		flow.vy = (int16_t)(flow_buf[22] + (flow_buf[23] << 8));
	}

	//debug[0] = flow.height;
	//debug[1] = flow.vx;
	//debug[2] = flow.vy;
	//debug[1] = flow.comp_x;
	//debug[2] = flow.comp_y;

}


#ifdef PX4FLOW
struct flow_stab stab;
uint8_t PID_Roll[3] = {80,40,30};//X
uint8_t PID_Pitch[3] = {80,40,30};//Y

void taskOptflow(void)
{	
	if(rcData[5] > 1800)//AUX-2
	{
		if((rcData[0] > 1450) && (rcData[0] < 1550) && (rcData[1] > 1450) && (rcData[1] < 1550))
		{
			static float old_error_vx,old_error_vy;
			float error_vx = -flow.vx * flow.height / 100;
			float error_vy = flow.vy * flow.height / 100;
			//debug[0] = error_vx;//+-100
			error_vx = constrainf(error_vx,-120,120);
			error_vy = constrainf(error_vy,-120,120);

			stab.error_vx_int += error_vx / 80;
			stab.error_vy_int += error_vy / 80;
			
			stab.error_vx_int = constrainf(stab.error_vx_int,-50,50);
			stab.error_vy_int = constrainf(stab.error_vy_int,-50,50);
			//debug[1] = stab.error_vx_int;//+-50

			//PID
			stab.cmd[0] = PID_Roll[0] * error_vx +
							PID_Roll[1] * stab.error_vx_int +
							PID_Roll[2] * constrainf((error_vx - old_error_vx),-100,100);
			//debug[2] = error_vx - old_error_vx;//+-100

			stab.cmd[1] = PID_Pitch[0] * error_vy +
							PID_Pitch[1] * stab.error_vy_int +
							PID_Pitch[2] * constrainf((error_vy - old_error_vy),-100,100);
			//debug[0] = error_vy - old_error_vy;//
			stab.cmd[0] = constrainf((stab.cmd[0] / 100),-120,120);
			stab.cmd[1] = constrainf((stab.cmd[1] / 100),-120,120);
			old_error_vx = error_vx;
			old_error_vy = error_vy;

			
			debug[0] = stab.cmd[0];//roll
			debug[1] = stab.cmd[1];//pitch
		}
		else 
		{
		stab.cmd[0] = 0;
		stab.cmd[1] = 0;
		stab.error_vx_int = 0;
		stab.error_vy_int = 0;
		}
	}
	else 
	{
		stab.cmd[0] = 0;
		stab.cmd[1] = 0;
		stab.error_vx_int = 0;
		stab.error_vy_int = 0;
	}

}
#endif

        
                
