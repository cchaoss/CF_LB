#ifndef _NRF2401_H_
#define _NRF2401_H_


//*********************************************NRF24L01*******************
#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width

#define RX_PLOAD_WIDTH  32  	// 32 uints TX payload
#define TX_PLOAD_WIDTH  32  	// 32 uints TX payload
//***************************************NRF24L01寄存器指令****************
#define NRF_READ_REG    0x00  	// 读寄存器指令
#define NRF_WRITE_REG   0x20 	// 写寄存器指令
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留
//*************************************SPI(nRF24L01)寄存器地址**************
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
//************************************************************************
#define RX_DR			6	  //中断标志
#define TX_DS			5
#define MAX_RT	  		4
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断


#define	BLACK 	0
#define WHITE 	1
#define RED 	2
#define ORANGE 	3
#define YELLOW 	4	
#define GREEN 	5
#define BLUE 	6
#define PINK 	7
#define VIOLET 	8

//mspCmd
#define RCDATA		1<<0
#define ARM			1<<1
#define FREEHEAD	1<<2
#define MANUAL		1<<3
#define ALTHOLD		1<<4
#define POSHOLD		1<<5
#define ONLINE		1<<6
#define CALIBRATION	1<<7
#define NEWADDRESS	1<<8
#define OFFLINE		1<<9
//dir
#define	UP		    1
#define	DOWN		2
#define	LEFT		3
#define	RIGHT		4
#define	FORWARD		5
#define	BACKWARD	6
#define	CR			7
#define	CCR			8

typedef struct _dataPackage
{
	uint8_t checkCode[4];
	uint8_t length;
	uint16_t mspCmd;
	uint16_t pitch;
	uint16_t roll;
	uint16_t throttle;
	uint16_t yaw;
	//int16_t X;
	//int16_t Y;
	//int16_t Z;
	uint16_t motor[4];
	uint8_t led;
	uint8_t dir;
	uint8_t dirdata;
	char trim_pitch;
	char trim_roll;
	uint8_t beep;
	
}dataPackage;

extern bool batt_low;
extern float height;
extern uint16_t batt;
extern dataPackage mspData;
extern int16_t roll1,pitch1,yaw1;

//328 data flag
enum _CMD
{
	ARM_P = 1,
	DISARM_P,
	CALIBRAT_P,
	ALTHOLD_P,
	NALTHOLD_P,
	LED_AON,
	LED_AOFF,
	LED_BON,
	LED_BOFF,
	LED_CON,
	LED_COFF,
	LED_DON,
	LED_DOFF,
	LED_AL_ON,
	LED_AL_OFF,
	RGBB_BLAC,
	RGBB_WHIT,
	RGBB_RED,
	RGBB_GREE,
	RGBB_BLUE,
	RGBB_ORAN,
	RGBB_YELL,
	RGBB_PINK,
	RGBB_VIOL,
	BEEP_OPEN,
	BEEP_STOP,
	BEEP_S,
	BEEP_M,
	BEEP_L,

	LEFT_P,
	RIGHT_P,
	FORWARD_p,
	BACK_P,
	UP_P,
	CR_P,
	CCR_P,
	TRIM_ROLL,
	TRIM_PITCH,
	MOTOR0,
	MOTOR1,
	MOTOR2,
	MOTOR3,
};


#define LED_A_ON	GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LED_A_OFF	GPIO_ResetBits(GPIOB, GPIO_Pin_3)

#define LED_B_ON	GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define LED_B_OFF	GPIO_ResetBits(GPIOB, GPIO_Pin_4)

#define LED_C_ON	GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define LED_C_OFF	GPIO_ResetBits(GPIOB, GPIO_Pin_5)

#define LED_D_ON	GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define LED_D_OFF	GPIO_ResetBits(GPIOB, GPIO_Pin_2)

#define SPI_CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_1)//CE
#define SPI_CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define SPI_CSN_H()  GPIO_SetBits(GPIOB, GPIO_Pin_12)//NSS/CSN
#define SPI_CSN_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_12)

#define bound(val,max,min) ((val) > (max)? (max) : (val) < (min)? (min) : (val))

void send_328p_buf(uint8_t len, uint8_t *buf);
bool NRF_Write_Reg(uint8_t reg, uint8_t data);
bool NRF_Write_Buf(uint8_t reg, uint8_t *data, uint8_t length);
bool NRF_Read_Buf(uint8_t reg, uint8_t *data, uint8_t length);

bool NRF24L01_INIT(void);
bool NRF24L01_Check(void); 
void nrf24l01HardwareInit(void);
void led_beep_sleep(void);

void rx_data_process(int16_t *buf);
bool nrf_rx(void);
void SetRX_Mode(void);

void nrf_scheduler(int16_t *buf);
void nrf_tx(void);
void SetTX_Mode(void);


#endif

