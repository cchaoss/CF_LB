#ifndef _OPTFLOW_H_
#define _OPTFLOW_H_


struct flow_data{
short x;
short y;
float comp_x;
float comp_y;
float height;
};
extern struct flow_data flow;

struct flow_stab{
int16_t cmd[2];
float error_x;
float error_y;
float error_vx_int;
float error_vy_int;
};
extern struct flow_stab stab;

void flow_init(void);

#define Bound(val,min,max) ((val) > (max)? (max) : (val) < (min)? (min) : (val))
#endif
