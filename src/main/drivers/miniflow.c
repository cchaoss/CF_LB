#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "common/maths.h"
#include "common/axis.h"
#include <platform.h>
#include "config/parameter_group.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "flight/imu.h"
#include "drivers/dma.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "c05/miniflow.h"
#include "build/debug.h"
#include "flight/altitudehold.h"
#include "rx/rx.h"
#include "c05/rxdata.h"
#include "flight/pid.h"



#define ROLL 0
#define PITCH 1
#define LENGTH 10

#define POS_CALC_OUT_MAX 	30.0f   
#define POS_CALC_OUT_MIN 	-30.0f
#define POS_CONTROL_LIMIT_MAX 	30.0f
#define POS_CONTROL_LIMIT_MIN 	-30.0f

_PID_arg_st pos_pid_para;
_PID_arg_st vec_pid_para;

_PID_val_st pos_pid_value_x;
_PID_val_st pos_pid_value_y;
_PID_val_st vec_pid_value_x;
_PID_val_st vec_pid_value_y;

struct flow_integral_frame  flow_data_frame;
struct flow_float flow_dat;
struct angle_data angle;

float pos_x, pos_y;
float test_filter_x, test_filter_y;
float speed_x, speed_y;
float sum_flow_x, sum_flow_y;
float disx,disy,setx,sety;
float pos_pid_out_x, pos_pid_out_y;
float	position_err_x, position_err_y;
int16_t flow_cmd[3] = {0,0,0};

uint8_t uart_flag = 0;
uint8_t flow_buffer[LENGTH+1];
static serialPort_t *flowPort;

static int32_t MedianFilter_x(int32_t newSonarReading);
static int32_t MedianFilter_y(int32_t newSonarReading);

void Miniflow_init(void)
{
	PID_para_init();
	flowPort = uartOpen(USART3,Receive_Data,115200,MODE_RXTX,SERIAL_NOT_INVERTED);
}


//120HZ LOOP
void taskminiFlow(void)
{	
	static uint8_t loop_20hz;
	loop_20hz++;
	Send_Angle();
	if(loop_20hz == 5)
	{	
		loop_20hz = 0;
		if(pControlFlag->altHoldFlag && pFlightData->position.z > 10)
		{
			Flow_Duty(0.05);
			if((rcData[0]<1450) || (rcData[0]>1550) || (rcData[1]<1450) || (rcData[1]>1550))//有摇杆操作时，定点各个数据清零
			{
				flow_cmd[ROLL] = 0;
				flow_cmd[PITCH] = 0;
				sum_flow_x = 0;       
				sum_flow_y = 0;		
				pos_x = 0;
				pos_y = 0;
				PID_Value_reset(&pos_pid_value_x);
				PID_Value_reset(&pos_pid_value_y);
				PID_Value_reset(&vec_pid_value_x);
				PID_Value_reset(&vec_pid_value_y);	
			}
		}
		else 
		{
			flow_cmd[ROLL] = 0;
			flow_cmd[PITCH] = 0;
			sum_flow_x = 0;       
			sum_flow_y = 0;		
			pos_x = 0;
			pos_y = 0;
			PID_Value_reset(&pos_pid_value_x);
			PID_Value_reset(&pos_pid_value_y);
			PID_Value_reset(&vec_pid_value_x);
			PID_Value_reset(&vec_pid_value_y);
		}

debug[2] = flow_cmd[PITCH];
debug[3] = flow_cmd[ROLL];
//debug[2] = pScratchData->position.x;
//debug[3] = pScratchData->position.y;
	}
}

void Flow_Duty(float dT)
{
	static uint8_t pos_flag = 0;
	if(1 == uart_flag)//如果接收完成，则转换更新数据
	{
		uart_flag = 0;
		pos_flag++;				
			
		flow_Decode(flow_buffer);
			
		test_filter_y = flow_dat.y*25.0f;
		test_filter_x = flow_dat.x*25.0f;
		LPF_1_(3.0f,dT,test_filter_y,speed_y);//pid控制中，如果使用了d，需要选择对速度进行低通滤波，否则飞机会抖动较大
		LPF_1_(3.0f,dT,test_filter_x,speed_x);   
//pFlightData->position.x = speed_x;
//pFlightData->position.y = speed_y;
		speed_y = constrainf(speed_y,-25.0f,25.0f);//30//右-左+
		speed_x = constrainf(speed_x,-25.0f,25.0f);//前-后+

		if(fabs(pScratchData->position.y - disx) > 0)//执行路径移动时应该清除PID的积分项
		{
			disx -= speed_x*0.05f;//往y为+ 35~=50cm 
			if(pScratchData->position.y - disx > 5)	setx = 25;//set speed
			else if(pScratchData->position.y - disx < -5) setx = -25;	
			else 
			{
				pScratchData->position.y = 0;
				setx = 0;
				disx = 0;
			}
		}

		if(fabs(pScratchData->position.x - disy) > 0)
		{
			disy -= speed_y*0.05f;//往x为+ 35~=50cm 
			if(pScratchData->position.x - disy > 5)	sety = 25;//set speed
			else if(pScratchData->position.x - disy < -5) sety = -25;	
			else
			{
				pScratchData->position.x = 0;
				sety = 0;
				disy = 0;
			}
		}
//debug[0] = disx;
//debug[1] = disy;
/*
	static float vel_x, vel_y;
	float dt;
	float vel_acc_x, vel_acc_y;
	float acc_tmp_x, acc_tmp_y;
		//加速度数据融合(加速度双重积分的数据滞后，实际飞行效果变差)
    	dt = accTimeSum * 1e-6f;
    	if (accSumCount) 
		{
			acc_tmp_x = (float)accSum[0] / (float)accSumCount;//pitch
			acc_tmp_y = (float)accSum[1] / (float)accSumCount;//roll
		}
    	else {acc_tmp_x = 0; acc_tmp_y = 0;}
    
    	vel_acc_x = acc_tmp_x * accVelScale * (float)accTimeSum;
		vel_acc_y = acc_tmp_y * accVelScale * (float)accTimeSum;

		pos_x += (vel_acc_x * 0.5f) * dt + vel_x * dt;                  
		pos_y += (vel_acc_y * 0.5f) * dt + vel_y * dt;                                               
    	pos_x = pos_x * 0.5f + sum_flow_x * 0.5f;                                                                   
    	pos_y = pos_y * 0.5f + sum_flow_y * 0.5f;  
		pos_x = constrainf(pos_x,POS_CONTROL_LIMIT_MIN,POS_CONTROL_LIMIT_MAX);
		pos_y = constrainf(pos_y,POS_CONTROL_LIMIT_MIN,POS_CONTROL_LIMIT_MAX);
//debug[2] = pos_y;

    	vel_x += vel_acc_x;
		vel_y += vel_acc_y;
		vel_x = vel_x * 0.5f + speed_x * 0.5f;
		vel_y = vel_y * 0.5f + speed_y * 0.5f;
		vel_y = constrainf(vel_y,-30.0f,30.0f);
		vel_x = constrainf(vel_x,-30.0f,30.0f);
debug[2] = speed_y;
debug[3] = vel_y;
*/
		//speed_x += setx;
		//speed_y += sety;
		sum_flow_y += (speed_y + sety)*0.04f;
		sum_flow_x += (speed_x + setx)*0.04f;//位移 cm
		sum_flow_y = constrainf(sum_flow_y,POS_CONTROL_LIMIT_MIN,POS_CONTROL_LIMIT_MAX);
		sum_flow_x = constrainf(sum_flow_x,POS_CONTROL_LIMIT_MIN,POS_CONTROL_LIMIT_MAX);

		if(2 == pos_flag)//光流每更新两次执行一次外环，也就是说，位置环的控制周期要大于速度环的控制周期，输出为期望速度
		{
			pos_flag = 0;
			pos_pid_out_y = PID_calculate(dT,0.0f,0.0f,sum_flow_y,&pos_pid_para,&pos_pid_value_y, POS_CALC_OUT_MAX);
			pos_pid_out_x = PID_calculate(dT,0.0f,0.0f,sum_flow_x,&pos_pid_para,&pos_pid_value_x, POS_CALC_OUT_MAX);

			pos_pid_out_y = constrainf(pos_pid_out_y*15.0f,POS_CALC_OUT_MIN,POS_CALC_OUT_MAX);
			pos_pid_out_x = constrainf(pos_pid_out_x*15.0f,POS_CALC_OUT_MIN,POS_CALC_OUT_MAX);

			if((pos_pid_out_x < 4.0f) && (pos_pid_out_x > -4.0f)) pos_pid_out_x = 0;
			if((pos_pid_out_y < 4.0f) && (pos_pid_out_y > -4.0f)) pos_pid_out_y = 0;
		}

	}		
	flow_cmd[ROLL] = PID_calculate(dT,0.0f,pos_pid_out_y,speed_y + sety,&vec_pid_para,&vec_pid_value_y,15.0f)*7.0f;//速度环
	flow_cmd[PITCH] = PID_calculate(dT,0.0f,pos_pid_out_x,speed_x + setx,&vec_pid_para,&vec_pid_value_x,15.0f)*7.0f;

	flow_cmd[ROLL] = -constrain(flow_cmd[ROLL],-35,35);//35
	flow_cmd[PITCH] = -constrain(flow_cmd[PITCH],-35,35);
}


float PID_calculate(float T, float in_ff, float expect, float feedback, _PID_arg_st *pid_arg, _PID_val_st *pid_val, float inte_lim)
{
	float out,differential;
	pid_arg->k_inc_d_norm = constrainf(pid_arg->k_inc_d_norm,0,1);
	
	pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	
	pid_val->err =  (expect - feedback);	
	
	pid_val->err_d = (pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	
	differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
	
	if((int16_t)(100 *pid_arg->inc_hz)!=0)	LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf);
		else	pid_val->err_d_lpf = 0;
	
	pid_val->err_i += (pid_val->err + pid_arg->k_pre_d *pid_val->feedback_d)*T;
	pid_val->err_i = constrainf(pid_val->err_i,-inte_lim,inte_lim);
	
	out = pid_arg->k_ff *in_ff + \
	    	pid_arg->kp *pid_val->err + \
	    	pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential + \
    		pid_arg->ki *pid_val->err_i;

	pid_val->feedback_old = feedback;
	pid_val->err_old = pid_val->err;
	
	return (out);
}

#define DISTANCE_SAMPLES_MEDIAN 3

static int32_t MedianFilter_x(int32_t newSonarReading)
{
    static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;
 
    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    sonarFilterSamples[currentFilterSampleIndex] = newSonarReading;
    currentFilterSampleIndex = nextSampleIndex;
    
    if (medianFilterReady)
        return quickMedianFilter3(sonarFilterSamples);
    else
        return newSonarReading;
}

static int32_t MedianFilter_y(int32_t newSonarReading)
{
    static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;
 
    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    sonarFilterSamples[currentFilterSampleIndex] = newSonarReading;
    currentFilterSampleIndex = nextSampleIndex;
    
    if (medianFilterReady)
        return quickMedianFilter3(sonarFilterSamples);
    else
        return newSonarReading;
}


void PID_para_init(void)
{
	pos_pid_para.kp = pidProfile()->P8[PIDPOSR]/1000.0f;
	pos_pid_para.ki = pidProfile()->I8[PIDPOSR]/1000.0f;
	pos_pid_para.kd = pidProfile()->D8[PIDPOSR]/1000.0f;
	pos_pid_para.k_ff = 0.01f;

	vec_pid_para.kp = pidProfile()->P8[PIDNAVR]/1000.0f;
	vec_pid_para.ki = pidProfile()->I8[PIDNAVR]/1000.0f;
	vec_pid_para.kd = pidProfile()->D8[PIDNAVR]/1000.0f;
	vec_pid_para.k_ff = 0.01f;
}

void PID_Value_reset(_PID_val_st *pid_val)
{
	pid_val->err = 0;
	pid_val->err_old = 0;
	pid_val->feedback_old = 0;
	pid_val->feedback_d = 0;
	pid_val->err_d = 0;
	pid_val->err_d_lpf = 0;
	pid_val->err_i = 0;
	pid_val->ff = 0;
	pid_val->pre_d = 0;
}


void Receive_Data(uint16_t data)
{
	static uint8_t count = 0,i = 0;

	switch(count)
	{
		case 0: if(data == 0xFE)
				{
					count = 1;
					uart_flag = 0;
				}
				break;
		case 1: if(data == 0x0A)
					count = 2;
				else 
					count = 0;
				break;
		case 2: if(i < LENGTH)	
					flow_buffer[i++] = data;
				else 
				{
					if(data == (flow_buffer[0]^flow_buffer[1]^flow_buffer[2]^flow_buffer[3]^flow_buffer[4]^ \
												flow_buffer[5]^flow_buffer[6]^flow_buffer[7]^flow_buffer[8]^flow_buffer[9]))
						uart_flag = 1;			

					i = 0;
					count = 0;
				}
				break;
		default:count = 0;i = 0;break;
	}
}


void flow_Decode(const uint8_t* f_buf)
{	
	uint16_t height = altitudeHoldGetEstimatedAltitude();//cm 
	if(height < 80) height = 100;
	else if(height < 130) height = 160;
			else height = 220; 

	if(f_buf[8]!= 0)
	{
  		flow_data_frame.pixel_flow_x_integral = f_buf[0] + (f_buf[1]<<8);
  		flow_data_frame.pixel_flow_y_integral = f_buf[2] + (f_buf[3]<<8);
 	}

	flow_data_frame.pixel_flow_x_integral = MedianFilter_x(flow_data_frame.pixel_flow_x_integral);
	flow_data_frame.pixel_flow_y_integral = MedianFilter_y(flow_data_frame.pixel_flow_y_integral);	
	//deadband
	if((flow_data_frame.pixel_flow_x_integral < 24) && (flow_data_frame.pixel_flow_x_integral > -24)) flow_data_frame.pixel_flow_x_integral = 0;//raw = 28
	if((flow_data_frame.pixel_flow_y_integral < 24) && (flow_data_frame.pixel_flow_y_integral > -24)) flow_data_frame.pixel_flow_y_integral = 0;	

	flow_dat.x = flow_data_frame.pixel_flow_x_integral*height/10000.0f;
	flow_dat.y = flow_data_frame.pixel_flow_y_integral*height/10000.0f;
}


void Send_Angle(void)
{
	static uint8_t DATA[8] = {0x4A,0,0,0,0,0,0};
	DATA[2] = angle.rol;
	DATA[3] = angle.rol>>8;
	DATA[4] = angle.pit;
	DATA[5] = angle.pit>>8;
	DATA[6] = angle.yaw;
	DATA[7] = angle.yaw>>8;
	DATA[1] = DATA[2]^DATA[3]^DATA[4]^DATA[5]^DATA[6]^DATA[7];
	serialWriteBuf(flowPort,DATA,8);
}


