#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <platform.h>
#include "nrf2401.h"
#include "stdio.h"
#include "bus_spi.h"
#include "bus_i2c.h"
#include "system.h"
#include "gpio.h"
#include "sound_beeper.h"
#include "io/beeper.h"
#include "build/debug.h"

golbal_flag flag = {"EMT",103,0,0,0,0,0,0,0,0,true};
package_328p msp_328p;
dataPackage mspData;
dataPackage t_mspData;

uint8_t  TXData[TX_PLOAD_WIDTH];//tx_data
uint8_t  TX_ADDRESS[RX_ADR_WIDTH]= {0x11,0xff,0xff,0xff,0xff};//tx_address

uint8_t  RXDATA[RX_PLOAD_WIDTH];//rx_data
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0x11,0xff,0xff,0xff,0xff};//rx_address


void send_328p_buf(uint8_t len, uint8_t *buf)
{
	while(len)
	{
		i2cWrite(0x08,0,*buf);
		delayMicroseconds(8);
		buf++;
		len--;
	}
}

bool NRF_Write_Reg(uint8_t reg, uint8_t data)
{
    SPI_CSN_L();
    spiTransferByte(SPI2, reg + 0x20);
    spiTransferByte(SPI2, data);
    SPI_CSN_H();
    return true;
}

bool NRF_Write_Buf(uint8_t reg, uint8_t *data, uint8_t length)
{
    SPI_CSN_L();
    spiTransferByte(SPI2, reg + 0x20);
    spiTransfer(SPI2, NULL, data, length);
    SPI_CSN_H();
    return true;
}

bool NRF_Read_Buf(uint8_t reg, uint8_t *data, uint8_t length)
{
    SPI_CSN_L();
    spiTransferByte(SPI2, reg); // read transaction
    spiTransfer(SPI2, data, NULL, length);
    SPI_CSN_H();
    return true;
}


/****************NRF24L01_Receive*********************/
bool nrf_rx(void)
{
    uint8_t sta;
    static uint8_t count;
    NRF_Read_Buf(NRFRegSTATUS, &sta, 1);
    if(sta & (1<<RX_DR)){
		//LED_C_ON;//
        NRF_Read_Buf(RD_RX_PLOAD,RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		memcpy(&t_mspData,RXDATA,sizeof(t_mspData));
		if(!(t_mspData.mspCmd & OFFLINE))
			mspData = t_mspData;
		else if(!(mspData.mspCmd & OFFLINE)){
				mspData.mspCmd |= OFFLINE;
				mspData.motor[ROL] = 1500;
				mspData.motor[PIT] = 1500;
				mspData.motor[YA ] = 1500;
				mspData.motor[THR] = 1000;
			 }
		NRF_Write_Reg(NRFRegSTATUS, sta);//清除nrf的中断标志位
		//NRF_Write_Reg(FLUSH_RX - 0X20,0xff);//
		count = 0;
     }
	else {
		count++;
		//LED_C_OFF;//
	}
	if(count > 45){//判断2.4G数据是否丢失
		count = 45;
		//beeper(3);//rc_lost_beep
		return false;
	}else return true;
}

void rx_data_process(int16_t *buf)
{
	static uint8_t x = 0;
	static bool arm_flag = false,roll_flag = false;
	if(!strcmp("$M<",(char *)mspData.checkCode)){
		if(mspData.mspCmd & ARM){//低电压不可以解锁，开机检测遥控为解锁状态需再次解锁
			if(arm_flag && roll_flag)	mwArm();
				else  mwDisarm();
			if(fabs(flag.pitch1) > 650 || fabs(flag.roll1) > 650){//侧翻保护
				mwDisarm();roll_flag = false;
			}
		}		
		else{	
			mwDisarm();
			roll_flag = true;
			if(flag.batt < 100) arm_flag = false;
				else arm_flag = true;
		}
		if(mspData.mspCmd & CALIBRATION){
			accSetCalibrationCycles(400);//mspData.mspCmd &= ~CALIBRATION;
		}

#if 1
		//offline process
		if(mspData.mspCmd & OFFLINE){
			LED_A_ON;
			i2cRead(0x08,0xff,1, &msp_328p.cmd);//debug[0] = msp_328p.cmd;
			i2cRead(0x08,0xff,1, &msp_328p.length);//debug[1] = msp_328p.length;
			for(uint8_t i = 0;i < msp_328p.length;i++)	
				i2cRead(0x08,0xff,1, &msp_328p.data[i]);
			//debug[2] = msp_328p.data[0];

			if(msp_328p.cmd == 255 && t_mspData.key != 0)
					i2cWrite(0x08,0,t_mspData.key);
		
			switch(msp_328p.cmd){  
				case ARM_P:mspData.mspCmd |= ARM;break;
				case ARM_OFF:mspData.mspCmd &= ~ARM;break;
				case CAL_P:mspData.mspCmd |= CALIBRATION;break;
				case ALT_P:mspData.mspCmd |= ALTHOLD;break;
				case ALT_OFF:mspData.mspCmd &= ~ALTHOLD;break;
				case LED_P:mspData.led = msp_328p.data[0];break;
				case LED_RGB:mspData.led_rgb = msp_328p.data[0];break;
				case BEEP_P:mspData.beep = msp_328p.data[0];break;
				case ROLL_P:mspData.motor[ROL] = 1500 + (msp_328p.data[0] > 100 ? 100 - msp_328p.data[0] : msp_328p.data[0]);break;
				case PITC_P:mspData.motor[PIT] = 1500 + (msp_328p.data[0] > 100 ? 100 - msp_328p.data[0] : msp_328p.data[0]);break;
				case YAW_P :mspData.motor[YA ] = 1500 + (msp_328p.data[0] > 100 ? 100 - msp_328p.data[0] : msp_328p.data[0]);break;
				case THRO_P:mspData.motor[THR] = 1100 + (flag.batt > 100 ? 124 - flag.batt : 0) * 12 + 4 * msp_328p.data[0];break;//Voltage compensation throttle
				case MOTOR_P:for(uint8_t i = 0;i < 4;i++)	mspData.motor[i] = 1000+4*msp_328p.data[i];break;
				default:break;
			}
		}
		else{
			LED_A_OFF;
			msp_328p.cmd = 255;
		}
		
		debug[3] = t_mspData.key;
#endif

		//give and bound the rc_stick data
		for(uint8_t i = 0;i<4;i++)	mspData.motor[i] = bound(mspData.motor[i],1950,1000);
		if(!((mspData.mspCmd & MOTOR) || (msp_328p.cmd == MOTOR_P)))//当要控制电机的时候，不把motor[]的值传给rcData
			for(uint8_t i = 0;i<4;i++)	buf[i] = mspData.motor[i];

		//just for beeper
		if(mspData.mspCmd & OFFLINE || mspData.mspCmd & ONLINE){
			if(mspData.beep == beep_off)BEEP_OFF;
			if(mspData.beep == beep_s){
				x++;
				if(x < 10) BEEP_ON;
				else if(x < 120) BEEP_OFF;else x = 0;
			}
			else if(mspData.beep == beep_m){
				x++;
				if(x < 40) BEEP_ON;
				else if(x < 120) BEEP_OFF;else x = 0;
			}
			else if(mspData.beep == beep_l){
				x++;
				if(x < 70) BEEP_ON;
				else if(x < 120) BEEP_OFF;else x = 0;
			}else x = 0;
			if(mspData.beep == beep_on)BEEP_ON;
		}
	}
}


/****************NRF24L01_TX*********************/
void nrf_tx(void)
{
	bool a = true;
	uint8_t sta;

	flag.cmd = mspData.mspCmd;
	flag.key = mspData.key;
	memcpy(TXData,&flag,sizeof(flag));
	
	SPI_CE_L();
	NRF_Write_Buf(WR_TX_PLOAD - 0x20,TXData,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	SPI_CE_H();//启动发送
	flag.yaw1 = 0;
	//while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0));

	while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) && a){
		delayMicroseconds(10);
		flag.yaw1++;
		if(flag.yaw1 > 700)a = false;
	}

	NRF_Read_Buf(NRFRegSTATUS,&sta,1); //读取状态寄存器的值	   
	NRF_Write_Reg(NRFRegSTATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta & MAX_TX)NRF_Write_Reg(FLUSH_TX - 0X20,0xff);//达到最大重发次数，清除TX FIFO寄存器
}


/************NFR24L01_Init************/
bool NRF24L01_INIT(void)
{
	uint8_t sta;
	nrf24l01HardwareInit();
	if(NRF24L01_Check()){
		SetRX_Mode();//default:0x11 ...
		for(uint8_t i = 0;i<5;i++){
			NRF_Read_Buf(NRFRegSTATUS, &sta, 1);
			delay(10);
		}
		if(sta & (1<<RX_DR)){
	        NRF_Read_Buf(RD_RX_PLOAD,RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer 
			memcpy(&mspData,RXDATA,sizeof(mspData));
			NRF_Write_Reg(NRFRegSTATUS, sta);//清除nrf的中断标志位
			if(mspData.mspCmd & NEWADDRESS){
				RX_ADDRESS[0] = mspData.motor[2];
				RX_ADDRESS[1] = mspData.motor[2] >> 8;
				RX_ADDRESS[2] = mspData.motor[3];
				RX_ADDRESS[3] = mspData.motor[3] >> 8;

				TX_ADDRESS[0] = mspData.motor[2] + 1;
				TX_ADDRESS[1] = mspData.motor[2] >> 8;
				TX_ADDRESS[2] = mspData.motor[3];
				TX_ADDRESS[3] = mspData.motor[3] >> 8;

				//save new_address to flash
				FLASH_Unlock();
				FLASH_ErasePage(0x0803E800);
				FLASH_ProgramWord(0x0803E800, mspData.motor[2]);
				FLASH_ProgramWord(0x0803E820, mspData.motor[3]);
				FLASH_Lock();
				SetRX_Mode();//use the new_address!
				for(uint8_t i = 0; i<5;i++)		{LED_D_ON;delay(50);LED_D_OFF;delay(50);}
			}
		}
		else{	//load the address form flash!
			RX_ADDRESS[0] = *(uint16_t *)0x0803E800;
			RX_ADDRESS[1] = *(uint16_t *)0x0803E800 >> 8;
			RX_ADDRESS[2] = *(uint16_t *)0x0803E820;
			RX_ADDRESS[3] = *(uint16_t *)0x0803E820 >> 8;

			TX_ADDRESS[0] = *(uint16_t *)0x0803E800 + 1;
			TX_ADDRESS[1] = *(uint16_t *)0x0803E800 >> 8;
			TX_ADDRESS[2] = *(uint16_t *)0x0803E820;
			TX_ADDRESS[3] = *(uint16_t *)0x0803E820 >> 8;

			SetRX_Mode();//use the new_address!
		}
		return true;
	}else return false;
}


bool NRF24L01_Check(void) 
{ 
	uint8_t buf = 0x77; 
   	uint8_t buf1; 
	
	NRF_Write_Buf(TX_ADDR,&buf,1); 
	delay(2);
	NRF_Read_Buf(TX_ADDR,&buf1,1); 

	if(buf1 == 0x77)return true;
		else	return false;
} 

void SetRX_Mode(void)
{
	SPI_CE_L();
	NRF_Write_Reg(FLUSH_RX,0xff);//清除TX FIFO寄存器			 
  	NRF_Write_Buf(RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
   	NRF_Write_Reg(EN_AA,0x01);       //使能通道0的自动应答    
  	NRF_Write_Reg(EN_RXADDR,0x01);   //使能通道0的接收地址  	 
  	NRF_Write_Reg(RF_CH,40);	 //设置RF通信频率		  
  	NRF_Write_Reg(RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF_Write_Reg(RF_SETUP,0x0f);   //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF_Write_Reg(CONFIG, 0x0f);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	SPI_CE_H();

} 

//发送模式
void SetTX_Mode(void)
{
	SPI_CE_L();
	NRF_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器
	NRF_Write_Buf(TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);	//写TX节点地址 
  	NRF_Write_Buf(RX_ADDR_P0,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); 	//设置TX节点地址,主要为了使能ACK	  
  	NRF_Write_Reg(EN_AA,0x01);     //使能通道0的自动应答    
  	NRF_Write_Reg(EN_RXADDR,0x01); //使能通道0的接收地址  
  	NRF_Write_Reg(SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF_Write_Reg(RF_CH,40);       
  	NRF_Write_Reg(RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF_Write_Reg(CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	SPI_CE_H();
  
} 

void nrf24l01HardwareInit(void)
{
	gpio_config_t IRQPIN;//nrf24l01 pins

	IRQPIN.pin = Pin_0;
	IRQPIN.mode = Mode_IN_FLOATING;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	gpioInit(GPIOB,&IRQPIN);


	gpio_config_t CE;//nrf24l01 pins

	CE.pin = Pin_1;
	CE.mode = Mode_Out_PP;
	CE.speed = Speed_10MHz;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	gpioInit(GPIOB,&CE);

	gpio_config_t LED;//init led pins

	LED.pin = Pin_3 | Pin_4 | Pin_5 | Pin_2;
	LED.mode = Mode_Out_PP;
	LED.speed = Speed_2MHz;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	gpioInit(GPIOB,&LED);
	
	LED_A_ON;delay(80);LED_A_OFF;
	LED_B_ON;delay(80);LED_B_OFF;
	LED_C_ON;delay(80);LED_C_OFF;
	LED_D_ON;delay(80);LED_D_OFF;
}

void led_beep_sleep(void)
{
	gpio_config_t led0;	

	led0.pin = Pin_11;
	led0.mode = Mode_AIN;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	gpioInit(GPIOB,&led0);

	gpio_config_t beep;	

	beep.pin = Pin_15;
	beep.mode = Mode_AIN;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	gpioInit(GPIOC,&beep);
}

