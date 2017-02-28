#include "stdio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <platform.h>
#include "system.h"
#include "gpio.h"
#include "led.h"
#include "nrf2401.h"

LED_t LEDCtrl;
//接口显存
LEDBuf_t LEDBuf;

/********************************************
              Led初始化函数
Led接口：
Led1-->PB2
Led2-->PB3
Led3-->PB4
Led4-->PB5
********************************************/
void LED_init(void)
{

	gpio_config_t LEDs;//init led pins

	LEDs.pin = Pin_2 | Pin_3 | Pin_4 | Pin_5 ;
	LEDs.mode = Mode_Out_PP;
	LEDs.speed = Speed_2MHz;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	gpioInit(GPIOB,&LEDs);
	

	//循环闪烁4次
    for(char i=0;i<4;i++) {
		LedA_on;LedB_off;LedC_off;LedD_off;
		delay(100);
		LedA_off;LedB_on;LedC_off;LedD_off;
		delay(100);
		LedA_off;LedB_off;LedC_on;LedD_off;
		delay(100);
		LedA_off;LedB_off;LedC_off;LedD_on;
		delay(100);
    }
	//解锁成功，快速闪烁3次提示
    for(char i=0;i<3;i++) {
		LedA_on;LedB_on;LedC_on;LedD_on;
		delay(100);
		LedA_off;LedB_off;LedC_off;LedD_off;
		delay(100);
    }

}

//底层更新 ，10Hz
void LEDReflash(void)
{
 
		if(LEDBuf.bits.A)
			LedA_on;
		else
			LedA_off;
		
		if(LEDBuf.bits.B)
			LedB_on;
		else
			LedB_off;
		
		if(LEDBuf.bits.C)
			LedC_on;
		else
			LedC_off;
		
		if(LEDBuf.bits.D)
			LedD_on;
		else
			LedD_off;

}

//事件驱动层
void LED_loop(void)
{
	//系统灯语
	LEDCtrl.event = READY;

	if(flag.batt_low)	LEDCtrl.event = BATL;
	
	if(flag.single_loss)	LEDCtrl.event = LOST;	

	if(flag.calibration)	LEDCtrl.event = CALI;
		
	switch(LEDCtrl.event) {
		case READY: if(++LEDCtrl.cnt >= 3)	LEDCtrl.cnt = 0;
					if(LEDCtrl.cnt == 0)	LEDBuf.byte = LA|LB;
					else	LEDBuf.byte = 0;
					break;
		case CALI :	LEDBuf.byte = LA|LB;
					break;
		case BATL : if(++LEDCtrl.cnt >= 3)	LEDCtrl.cnt = 0;
					if(LEDCtrl.cnt == 0)	LEDBuf.byte = 0x0f;
					else	LEDBuf.byte = 0;
					break;

		case LOST : if(++LEDCtrl.cnt >= 4)	LEDCtrl.cnt = 0;
					LEDBuf.byte = 1<<LEDCtrl.cnt ;
					break;	
	}

	LEDReflash();
}



