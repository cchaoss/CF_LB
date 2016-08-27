#ifndef _NRF2401_H_
#define _NRF2401_H_


//*********************************************NRF24L01*******************
#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width

#define RX_PLOAD_WIDTH  32  	// 32 uints TX payload
#define TX_PLOAD_WIDTH  32  	// 32 uints TX payload
//***************************************NRF24L01寄存器指令****************
#define NRF_READ_REG    0x00  	// 读寄存器指令
#define NRF_WRITE_REG   0x20 	  // 写寄存器指令
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	  // 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留
//*************************************SPI(nRF24L01)寄存器地址****************
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define NRFRegSTATUS    0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道1接收数据长度
#define RX_PW_P2        0x13  // 接收频道2接收数据长度
#define RX_PW_P3        0x14  // 接收频道3接收数据长度
#define RX_PW_P4        0x15  // 接收频道4接收数据长度
#define RX_PW_P5        0x16  // 接收频道5接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
//**************************************************************************************
#define RX_DR			6//中断标志
#define TX_DS			5
#define MAX_RT	  4

#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define MSP_SET_THRO  1
#define MSP_SET_YAW		2
#define MSP_SET_PITCH	3
#define MSP_SET_ROLL	4
#define MSP_ARM_IT		5
#define MSP_DISARM_IT	6
#define MSP_SET_4CON	7
#define MSP_SETOFF		8
#define MSP_LAND_DOWN 9
#define MSP_HOLD_ALT 	10
#define MSP_STOP_HOLD_ALT 11
#define MSP_HEAD_FREE 12
#define MSP_STOP_HEAD_FREE	13
#define MSP_POS_HOLD 14
#define MSP_STOP_POS_HOLD 15
#define MSP_FLY_STATE	16
#define MSP_ACC_CALI	205


#define SPI_CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_1)//CE
#define SPI_CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define SPI_CSN_H()  GPIO_SetBits(GPIOB, GPIO_Pin_12)//NSS/CSN
#define SPI_CSN_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_12)

extern bool tx_done;
extern int16_t roll1,pitch1,yaw1;

bool NRF_Write_Reg(uint8_t reg, uint8_t data);
bool NRF_Write_Buf(uint8_t reg, uint8_t *data, uint8_t length);
bool NRF_Read_Buf(uint8_t reg, uint8_t *data, uint8_t length);

bool NRF24L01_INIT(void);
bool NRF24L01_Check(void); 
void nrf24l01HardwareInit(void);

void Nrf_Irq(int16_t *buf);
void SetRX_Mode(void);

void NRF24L01_TXDATA(void);
void SetTX_Mode(void);

#endif



