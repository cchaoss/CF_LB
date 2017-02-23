#include "stdio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <platform.h>
#include "bus_spi.h"
#include "bus_i2c.h"
#include "system.h"
#include "gpio.h"
#include "nrf2401.h"
#include "app.h"
#include "build/debug.h"

golbal_flag flag = {"EMT",VerSion,0,0,0,0,0,0,0,0,true};
dataPackage mspData;

uint8_t  TXData[TX_PLOAD_WIDTH];//tx_data
uint8_t  TX_ADDRESS[RX_ADR_WIDTH]= {0x11,0xff,0xff,0xff,0xff};//tx_address

uint8_t  RXDATA[RX_PLOAD_WIDTH];//rx_data
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0x11,0xff,0xff,0xff,0xff};//rx_address


static inline void NRF_Write_Reg(uint8_t reg, uint8_t data)
{
    SPI_CSN_L();
    spiTransferByte(SPI2, reg + 0x20);
    spiTransferByte(SPI2, data);
    SPI_CSN_H();
}

static inline void NRF_Write_Buf(uint8_t reg, uint8_t *data, uint8_t length)
{
    SPI_CSN_L();
    spiTransferByte(SPI2, reg + 0x20);
    spiTransfer(SPI2, NULL, data, length);
    SPI_CSN_H();
}

static inline void NRF_Read_Buf(uint8_t reg, uint8_t *data, uint8_t length)
{
    SPI_CSN_L();
    spiTransferByte(SPI2, reg); // read transaction
    spiTransfer(SPI2, data, NULL, length);
    SPI_CSN_H();
}


/****************NRF24L01_Receive*********************/
bool nrf_rx(void)
{
    uint8_t sta;
	static uint8_t count;
    NRF_Read_Buf(NRFRegSTATUS, &sta, 1);
    if(sta & (1<<RX_DR)){
        NRF_Read_Buf(RD_RX_PLOAD,RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		memcpy(&mspData,RXDATA,sizeof(mspData));
		NRF_Write_Reg(NRFRegSTATUS, sta);//清除nrf的中断标志位
		count = 0;
     }else count++;

	if(count > 60) {
		count = 60;
		return false;
	}else return true;
}

void rx_data_process(int16_t *buf)
{
	static bool arm_flag = false,roll_flag = false;

	if(App_data_ok) {
		if(App_data[4] & APP_ARM) {
			mspData.mspCmd |= ARM;
			debug[0] = 100;
		}
		if(App_data[4] & APP_DIS) {
			mspData.mspCmd &= ~ARM;
			debug[0] = 1;
		}

		if(App_data[4] & APP_ALT) {
			mspData.mspCmd |= ALTHOLD;
			debug[1] = 100;
		}
		else { mspData.mspCmd &= ~ALTHOLD;debug[1] = 1;}

		if(App_data[4] & APP_CAL) {
			mspData.mspCmd |= CALIBRATION;
			debug[2] = 100;
		}
		else {mspData.mspCmd &= ~CALIBRATION;debug[2] = 1;}
	
		mspData.motor[0] = (App_data[0]<<2) + 988;
		mspData.motor[1] = (App_data[1]<<2) + 988;
		mspData.motor[2] = (App_data[3]<<2) + 988;
		mspData.motor[3] = (App_data[2]<<2) + 988;
	}

	if(!strcmp("$M<",(char *)mspData.checkCode) || App_data_ok){
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

		//give and bound the rc_stick data
		for(uint8_t i = 0;i<4;i++)	buf[i] = bound(mspData.motor[i],2000,1000);
	}
}


/*
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
*/

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
  	NRF_Write_Reg(RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   250kb+0dbm:0x26 1M+0dbm:0x06 
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

