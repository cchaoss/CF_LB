#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdint.h>
#include <platform.h>
#include "nrf2401.h"
#include "stdio.h"
#include "bus_spi.h"
#include "system.h"
#include "gpio.h"
#include "../target/SPRACINGF3/target.h"



uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];	//rx_data
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x01};		//rx_address


uint8_t SPI_RW(uint8_t dat) 
{

	spiTransferByte(SPI2,dat);
}


uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    SPI_CSN_L();					  
    status = SPI_RW(reg);  
    SPI_RW(value);		 
    SPI_CSN_H();					 
    return 	status;
}


uint8_t NRF_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    SPI_CSN_L();					 
    SPI_RW(reg);			  
    reg_val = SPI_RW(0);	
    SPI_CSN_H();	
 
    return 	reg_val;
}


uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    SPI_CSN_L();				        /* ???? */
    status = SPI_RW(reg);	/* ?????? */
    for(i=0; i<uchars; i++)
    {
        SPI_RW(pBuf[i]);		/* ??? */
    }
    SPI_CSN_H();						/* ????? */
    return 	status;	
}


uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    SPI_CSN_L();						/* ???? */
    status = SPI_RW(reg);	/* ?????? */
    for(i=0; i<uchars; i++)
    {
        pBuf[i] = SPI_RW(0); /* ?????? */ 	
    }
    SPI_CSN_H();						/* ????? */
    return 	status;
}


void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
    SPI_CE_L();		 //StandBy I??	
    NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // ????	
    SPI_CE_H();		 //??CE,??????
}


uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							   
        //SPI2_SetSpeed(SPI_SPEED_4); //spi???9Mhz(24L01???SPI???10Mhz)   
	sta=NRF_Read_Reg(NRFRegSTATUS);  //?????????    	 
	NRF_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,sta); //??TX_DS?MAX_RT????
	if(sta&RX_OK)//?????
	{
		NRF_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//????
		NRF_Write_Reg(FLUSH_RX,0xff);//??RX FIFO??? 
		return 0; 
	}	   
	return 1;//???????
}		


void Nrf_Irq(void)
{
    uint8_t sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
    if(sta & (1<<RX_DR))//???????
    {
        NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
        ReceiveDataFormNRF();    //?????
	NRF_Write_Reg(0x27, sta);//??nrf??????
	sta = 0;
    }
    
}
   
   
/*************************************************************/
//init 
char NRF24L01_INIT(void)
{
	//spi init
    spiSetDivisor(SPI2, SPI_0_5625MHZ_CLOCK_DIVIDER);//128DIV

	if(NRF24L01_Check() == 0)
	{
		SetRX_Mode();
		spiSetDivisor(SPI2, SPI_18MHZ_CLOCK_DIVIDER);  // 18 MHz SPI clock  2DIV
	}
	else return -1;

}


uint8_t NRF24L01_Check(void) 
{ 
   uint8_t buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2}; 
   uint8_t buf1[5]; 
   uint8_t i=0; 
    
   /*??5 ??????.  */ 
   NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5); 
     
   /*??????? */ 
   NRF_Read_Buf(TX_ADDR,buf1,5); 
   
    /*??*/ 
   for (i=0;i<5;i++) 
   { 
      if (buf1[i]!=0xC2) 
      return -1; 
   } 
  
//   if (i==5)   {printf("NRF24L01 found...\r\n");return 1 ;}        //MCU ?NRF ???? 
//   else        {printf("NRF24L01 check failed...\r\n");return 0 ;}        //MCU?NRF?????   
	return 0;
} 

void SetRX_Mode(void)
{
    SPI_CE_L();
	NRF_Write_Reg(FLUSH_RX,0xff);//??TX FIFO???			 
  	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//?RX????
   	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //????0?????    
  	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//????0?????  	 
  	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //??RF????		  
  	NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//????0??????? 	    
  	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//??TX????,0db??,2Mbps,???????   
  	NRF_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//???????????;PWR_UP,EN_CRC,16BIT_CRC,???? 
    SPI_CE_H();
    //printf("NRF24L01 Set to Receiving Mode,RX_ADDR 0x%x...\r\n",RX_ADDRESS[4]);
} 


