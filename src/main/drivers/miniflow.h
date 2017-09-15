#ifndef _miniflow_H
#define _miniflow_H

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) ) *( (in) - (out) ))	//一阶低通滤波
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )

struct flow_integral_frame {
    signed short pixel_flow_x_integral;
    signed short pixel_flow_y_integral;
    unsigned int integration_timespan;
    unsigned short ground_distance;
    unsigned char qual;
	unsigned char valid;	
};

struct flow_float{
	float x;
	float y;
	unsigned short dt;
	unsigned char qual;	
	unsigned char update;
};
extern struct flow_float flow_dat;

extern float exp_rol_flow,exp_pit_flow;

typedef struct
{
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd;		 	 //微分系数
	float k_pre_d; //previous_d 微分先行
	float inc_hz;  //不完全微分低通系数
	float k_inc_d_norm; //Incomplete 不完全微分 归一（0,1）
	float k_ff;		 //前馈 
}_PID_arg_st;

typedef struct
{
	float err;
	float err_old;
	float feedback_old;
	float feedback_d;
	float err_d;
	float err_d_lpf;
	float err_i;
	float ff;
	float pre_d;

}_PID_val_st;

struct angle_data
{
	int16_t rol;
	int16_t pit;
	int16_t yaw;
};
extern struct angle_data angle;

extern int16_t flow_cmd[3];

void Miniflow_init(void);
void Receive_Data(uint16_t data);
void flow_Decode(const unsigned char* f_buf);
void PID_para_init(void);
void PID_Value_reset(_PID_val_st *pid_val);
void Send_Angle(void);
void Flow_Duty(float dT);
float PID_calculate(float T, float in_ff, float expect, float feedback, _PID_arg_st *pid_arg, _PID_val_st *pid_val, float inte_lim);

#endif


