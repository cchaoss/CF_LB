#ifndef _OPTFLOW_H_
#define _OPTFLOW_H_


struct flow_data{
short vx;
short vy;
float comp_x;
float comp_y;
float height;
};
extern struct flow_data flow;

void flow_init(void);


#endif
