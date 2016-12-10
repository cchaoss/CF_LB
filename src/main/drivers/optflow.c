#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <platform.h>
#include "build/debug.h"
#include "optflow.h"

#include "config/parameter_group.h"
#include "drivers/dma.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "common/crc.h"

struct flow_data flow;

static void flowDataReceive(uint16_t data);

float ByteToFloat(uint8_t* byteArry)
{
  return *((float*)byteArry);
}

void flow_init(void)
{

	openSerialPort(SERIAL_PORT_UART2,FUNCTION_TELEMETRY_MAVLINK, flowDataReceive, 115200, MODE_RX, SERIAL_NOT_INVERTED);//SERIAL_STOPBITS_1
	
}

uint8_t flow_buf[30],flow_state[4];
void flowDataReceive(uint16_t data)
{
	static uint8_t check = 0,data_cnt=0;
	uint8_t temp[4];
	float *p = (float*)temp;
	bool Catch_flag = false;

	switch(check)
	{
   		case 0: if(data == 0xFE)check=1;break;
 		case 1: if(data == 0x1A)check=2;
				else check=0;
				break;
		case 2:	if(data_cnt < 4){
					check = 2;
					flow_state[data_cnt++] = data;
				}
				else{
					check = 3;
					data_cnt = 0;
					flow_buf[data_cnt++] = data;
				}
				break;
		case 3:	if(flow_state[3] == 100){
					if(data_cnt < 26){
						check = 3;
						flow_buf[data_cnt++] = data;
					}
                  	else{
						check=4;
						data_cnt=0;
					}
				}
				else{
					check = 0;
					data_cnt = 0;
				}
				break;
		case 4: check = 0;
				data_cnt = 0;
				Catch_flag = true;
				break;
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
		flow.vx = (int16_t)(flow_buf[20] + (flow_buf[21] << 8));
		flow.vy = (int16_t)(flow_buf[22] + (flow_buf[23] << 8));
	}
	debug[0] = flow.height;
	debug[1] = flow.comp_x;
	debug[2] = flow.comp_y;
	debug[3] = flow.vx;
	//debug[3] = flow.vy;

}



        
                
