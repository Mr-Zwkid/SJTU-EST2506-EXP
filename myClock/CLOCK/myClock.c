
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "pwm.h"
#include "timer.h"
#include "driverlib/rom.h"
#include "driverlib/flash.h"
#include "driverlib/rom_map.h"


#define SYSTICK_FREQUENCY		1000			//1000hz

#define	KEY_TIMER  				1000				//500mS
#define GPIO_FLASHTIME			300				//300mS

// 引脚端口的地址定义

#define TCA6424_I2CADDR 				0x22
#define PCA9557_I2CADDR					0x18

#define PCA9557_INPUT					0x00
#define	PCA9557_OUTPUT					0x01
#define PCA9557_POLINVERT				0x02
#define PCA9557_CONFIG					0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06

// 按键含义
# define  Key_Return	0x8
# define  Key_Date		0x1
# define  Key_Time		0x2
# define  Key_Set    	0x3
# define  Key_Music    	0x4
# define  Key_Null		0x0
# define  Key_Left		0x6
# define  Key_Right  	0x5
# define  Key_Enter 	0x7

void 		Delay(uint32_t value);
void 		LedLight(uint8_t pos);
void        DigitWrite( uint8_t *data, uint8_t n);
void     	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void 		RunningWaterStrProcess(void);

void 		S800_GPIO_Init(void);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
void 		SysTick_Init(void);
void		PWM_Init(void);
void		Timer1_Init(void);
void 		Device_Init(void);
void		BootScreen(void);
void        SetupForUART(void);

void        UARTStringPutNonBlocking(const char *cMessage);
void        UARTStringPut(const char *cMessage);
void        CharEcho(char ch, uint8_t cnt);
void 		UARTCmdProcess(void);
void		ErrorSend(void);
void		TimeSend(void);
void		DateSend(void);
void		AlarmSend(void);
void		HelpSend(void);


void 		KeyDecoder(void);
void		ModeSelect(void);
void        ClockGen( uint8_t h, uint8_t m, uint8_t s);
void        DateGen( uint16_t y, uint8_t m, uint8_t d);
void        AlarmGen( uint8_t h, uint8_t m, uint8_t s);
void		DisplayModeOn(void);
void 		SetModeOn(void);
void		SetTime(void);
void		SetDate(void);
void		SetAlarm(void);
void		SetMusicOn(void);
void		SetMusicOff(void);


// Systick软定时
volatile uint16_t   systick_1ms_couter ,systick_10ms_couter ,systick_250ms_couter = 0, systick_500ms_couter = 0, systick_1000ms_couter = 0;
volatile uint8_t	systick_1ms_status ,systick_10ms_status ,systick_250ms_status, systick_500ms_status, systick_1000ms_status;

volatile uint8_t gpio_status;
volatile uint16_t gpio_flash_cnt;	
uint32_t ui32SysClock;		// 系统时钟
uint8_t  DisplayStatus = 2;	// 数码管显示状态 1-日期，2-时间, 3-左流水慢, 4-右流水慢, 5-左流水快, 6-右流水快
uint8_t  seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x40};
uint8_t  ABCs[] = {0x77,0x7c,0x39,0x5e,0x79,0x71,0x3d,0x76,0x0f,0x0e,0x75,0x38,0x37,0x54,0x5c,0x73,0x67,0x31,0x49,0x78,0x3e,0x1c,0x7e,0x64,0x6e,0x59};


// UART有关变量
uint8_t  uart_receive_char;
uint8_t  uart_receive_status;
uint8_t  uart_char_buffer[200];
uint8_t  uart_char_curIdx = 0;

// 时间有关变量
uint8_t  hh = 12;      // 全局小时数
uint8_t  mm = 00;      // 全局分钟数
uint8_t  ss = 00;      // 全局秒数
uint8_t  h, m, s;     // 命令处理中暂存时间的变量
uint8_t  digits1[] = {'1', '2', '-', '0', '0', '-', '0', '0'}; // hh-mm-ss
uint8_t  timeStr[] = "12:00:00";  // 当前时间的字符串

// 日期有关变量
uint16_t  year = 2024;      // 全局年份数
uint8_t   mon = 06;         // 全局月份数
uint8_t   day = 03;         // 全局日数
uint16_t   y1, m1, d1;       // 命令处理中暂存日期的变量
uint8_t   digits2[] = {'2', '0', '2', '4', '0', '6', '0', '3'}; // year month day
uint8_t   dateStr[] = "2024-06-03";  // 当前日期的字符串

// 闹钟有关变量
uint8_t AlarmH;		// 闹钟小时
uint8_t AlarmM;		// 闹钟分钟
uint8_t AlarmS;		// 闹钟秒
uint8_t alarmStr[] = "07:00:00";

// 按键有关变量
uint8_t key_mode = 0;	// 按键模式
uint8_t key_status = 0; // 按键状态
uint8_t key_value = 0;	// 当前按键值

uint8_t key_right = 0;	// 右移按键检测
uint8_t key_left = 0;	// 左移按键检测
uint8_t key_enter = 0;	// 确认按键检测

uint8_t key_plus_status;  // 加按键状态
uint8_t key_plus;		  // 加按键检测
uint8_t key_minus_status; // 减按键状态
uint8_t key_minus;		  // 减按键检测

uint16_t key_left_timer = 0;	// 长短键识别
uint16_t key_right_timer = 0;

// 乐曲有关变量
uint16_t freqs[] = {
					988, 1046, 880, 0, 659, 698, 587, 0, 
					740, 831, 659, 659, 494, 523, 440, 494, 523, 440,
					988, 1046, 880, 0, 1318, 880, 988, 1046, 880, 0, 1318, 880, 
					988, 1046, 880, 880, 831, 880, 0, 988, 1046, 880,
					988, 1046, 880, 0, 1318, 880, 659, 698, 587, 0, 
					740, 831, 659, 659, 494, 523, 440, 494, 523, 440,
					988, 1046, 880, 0, 1318, 880, 988, 1046, 880, 0, 1318, 880, 
					988, 1046, 880, 880, 831, 880, 0, 988, 1046, 880,
					1046, 659, 880, 988, 1046, 659, 880, 1046, 1175, 659, 1046, 659, 988, 659, 880, 659, 
					988, 659, 831, 880, 988, 659, 784, 988, 1046, 659, 988, 659, 880, 659, 1318, 659,
					1046, 659, 880, 988, 1046, 659, 880, 1046, 1175, 659, 1046, 659, 988, 659, 880, 659, 
					1318, 659, 831, 659, 1046, 659, 988, 659, 880, 659, 1318, 659, 880,
					1318, 880, 1046, 1175, 1318, 880, 1046, 1568, 1397, 880, 1318, 880, 1175, 880, 1046, 880,
					1175, 784, 988, 1046, 1175, 784, 988, 1397, 1318, 659, 1175, 659, 1046, 659, 988, 659, 
					1046, 659, 880, 988, 1046, 659, 880, 1318, 1175, 659, 1046, 659, 988, 659, 880, 659,
					988, 659, 1046, 988, 880, 1318, 1760, 
					1318, 880, 1046, 1175, 1318, 880, 1046, 1568, 1397, 880, 1318, 880, 1397, 880, 1760, 880, 
					1568, 988, 1175, 1760, 1568, 988, 1397, 988, 1318, 659, 1175, 1318, 1046, 1318, 988, 1318, 
					1046, 659, 880, 988, 1046, 659, 880, 1318, 1175, 659, 1046, 659, 988, 659, 880, 659,
					988, 659, 831, 988, 1175, 784, 988, 1175, 1318, 988, 1175, 1318, 1661, 1175, 1318, 1568,
					1760,
					440, 523, 440, 587, 440, 523, 494, 440, 523, 440, 587, 440, 523, 
					0, 440, 523, 659, 440, 523, 440, 0, 622, 659, 880, 1046, 1175, 1046,
					659, 784, 659, 880, 659, 784, 587, 659, 784, 659, 880, 659, 784,
					988, 1046, 880, 0, 1318, 880, 988, 1046, 880, 0, 1318, 880, 
					988, 1046, 880, 880, 831, 880, 0, 988, 1046, 880,
					988, 1046, 880, 0, 1318, 880, 988, 1046, 880, 0, 1318, 880, 
					988, 1046, 880, 880, 831, 880, 0, 988, 1046, 880
					};
float time[] = {
				0.25, 0.25, 0.5, 1, 0.25, 0.25, 0.5, 1, 
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.25, 0.25, 1,
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.25, 0.25, 1,
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.5, 1, 
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.25, 0.25, 1,
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.25, 0.25, 1,
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 1,
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1,
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 
				4,
				0.25, 0.5, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.5, 0.25, 0.25, 0.25, 0.5,
				0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.5, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.5,
				0.25, 0.5, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.5, 0.25, 0.25, 0.25, 0.5,
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.25, 0.25, 1,
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 
				0.25, 0.25, 0.5, 0.5, 0.25, 0.25, 0.5, 0.25, 0.25, 1,
				};
uint16_t SizeOfSong = 347; 	  // 乐曲数组总长度
uint16_t CurPos = 0; 	      // 当前位置
uint16_t BeatTime = 536; 	  // 每拍时间（ms）
uint8_t  MusicOn = 0;		  // 音乐是否响起


uint32_t temp;
uint8_t RunningIdx = 0;		// 流水显示的当前位置
uint8_t fullStr[27];		// 流水显示的字符串
uint8_t	LedPos = 1;

int main(void)
{
	Device_Init();

	BootScreen();
	
	while (1)
	{
		ModeSelect();

		// 指令字符串处理
        if (uart_receive_status)
        {
            uart_receive_status = 0;
			UARTCmdProcess();
        }

		// 数码管和键值更新、流水显示
        if (systick_1ms_status)
        {
            systick_1ms_status = 0;

			// 数码管显示更新
			DisplayModeOn();

			// 按键值更新
			KeyDecoder();
        }

		// 非设置模式下，正常计时
		if (key_mode != Key_Set && systick_1000ms_status)
		{
			systick_1000ms_status = 0;

			// 到达闹钟时间，开启音乐播放
			if (hh == AlarmH && mm == AlarmM && ss == AlarmS){
				SetMusicOn();
			}

			ClockGen(hh, mm, ss + 1);
			DateGen(year, mon, day);
			RunningWaterStrProcess();
		}
	}
}

/* 工具函数 */ 

// 延时
void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

// 数码管写入数字，从左到右为0~7位，n为显示的位数
void DigitWrite( uint8_t *data, uint8_t n){
	uint8_t i;
    uint8_t tmp;
	for (i = 0; i < n; i++){
		if ((data[i] - '0' >= 0 && data[i] - '0' <= 9) || data[i] == '-') {
			tmp = (data[i] != '-') ? data[i] - '0' : 10;
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1, seg7[tmp]);	
			// 显示的位置，高有效								
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, 0x1 << i);
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, 0x0);
		}	
	}
}


// 点亮蓝板对应位置的LED，高有效，从左到右为0~7位
void  LedLight(uint8_t pos)
{
	I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~pos); // LED 低有效 从左到右为0~7位
}

// 获取流水显示的字符串
void  RunningWaterStrProcess(void)
{
	fullStr[0] = '\0';
	strcpy((char*)fullStr, (const char*) digits2);
	fullStr[8] = ' ';
	fullStr[9] = '\0';
	strcat((char*)fullStr, (const char*) digits1);
	fullStr[17] = ' ';
	fullStr[18] = '\0';
	strcat((char*)fullStr, (const char*) digits2);
	fullStr[26] = ' ';
}

// I2C写入数据
void I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{

	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	while(I2CMasterBusy(I2C0_BASE)){};

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
}

// I2C读取数据
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value;

	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);

	while(I2CMasterBusBusy(I2C0_BASE));
	Delay(1000);

	// receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

	while(I2CMasterBusBusy(I2C0_BASE));
	value = I2CMasterDataGet(I2C0_BASE);
	Delay(1000);

	return value;
}



/* 初始化函数 */

// GPIO 初始化
void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			//Set PN0 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);			//Set PN1 as Output pin

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

// I2C 初始化
void S800_I2C0_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);	 //config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);				//config port as output
	I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}


// UART 初始化
void S800_UART_Init(void)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);				
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			

	GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);    			

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTStringPut((const char *)"\r\nDigital Clock Initializing...\r\n\n");
	UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX2_8, UART_FIFO_RX7_8);

	// UART0 中断允许
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

// PWM 初始化
void  PWM_Init(void)
{
	// PWM0使能
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	// 使能端口K
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));

	// 配置PK5为M0PWM7
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinConfigure(GPIO_PK5_M0PWM7);

	// 引脚映射
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);

	// 配置PWM发生器3
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	// 使能PWM输出口7
	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
}

// Timer1 初始化
void Timer1_Init(void)
{
	// TIMER1 使能
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); 

	// 设置为 32 位周期定时器
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); 

	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock); // TIMER1A 装载计数值 1s 
	TimerEnable(TIMER1_BASE, TIMER_A); // TIMER1 开始计时

	// Timer1 中断允许
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Timer1A 超时中断使能
	IntEnable(INT_TIMER1A); // 开启 TIMER1A 中断源
}

// SysTick 初始化
void SysTick_Init(void)
{
	// Systick 使能
	SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);

	// Systick 中断允许
	SysTickEnable();
	SysTickIntEnable();

}

// 所有设备初始化
void Device_Init(void)
{
	// 系统时钟设置
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 20000000);

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	SysTick_Init();
	PWM_Init();
	Timer1_Init();

	// 总中断允许
    IntMasterEnable();	
	
	// 中断优先级设置
	IntPriorityGroupingSet(3);		// 设置高3位全部用于抢占优先级设置，越小越优先
	IntPrioritySet(INT_UART0,0x0e0);	
	IntPrioritySet(FAULT_SYSTICK,0x03);

}

// 开机画面 / 初始化
void BootScreen(void)
{
	uint8_t StudentId[] = {2, 1, 9, 1, 0, 1, 2, 1};
	uint8_t StudentName[] = {'Z', 'H', 'A', 'N', 'G', 'W', 'E', 'N', 'K', 'A', 'N', 'G'};
	uint8_t i;

	// LED全亮全灭一次
	LedLight(0xff);
	Delay(ui32SysClock / 20);
	LedLight(0x00);
	Delay(ui32SysClock / 20);

	// 学号后8位：21910121
	for (i = 0; i < 8; i++){
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1, seg7[StudentId[i]]);								
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, 0x1 << i);
		Delay(ui32SysClock / 20);
	}

	// 姓名字母 ZHANG WEN KANG
	for (i = 0; i < 12; i++){
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1, ABCs[StudentName[i] - 'A']);								
		I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, 0x1 << (i % 8));
		Delay(ui32SysClock / 20);
	}

	// 串口提示信息
	CharEcho('*', 50);
	UARTStringPut("Hello! This is a Digital Clock by ZWK...\n\n");
	UARTStringPut("Enter \'HELP\' to See CMDs...\n\n");
	UARTStringPut("NOTE: THE FIRST CMD SHOULD ADD A BLANK AT THE FRONT...\n\n");
	UARTStringPut("HAVE FUN...\n");
	CharEcho('*', 50);

	// 初始化时钟、日期、闹钟
	ClockGen(15, 00, 00);
	DateGen(2024, 6, 15);
	AlarmGen(7, 0, 0);
}



/* UART 通信 */

void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}

void UARTStringPutNonBlocking(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPutNonBlocking(UART0_BASE,*(cMessage++));
}

void  CharEcho(char ch, uint8_t cnt)
{
	uint8_t i;
	for (i = 0; i < cnt; i++)
		UARTCharPut(UART0_BASE, ch);
	
	UARTCharPut(UART0_BASE,'\n');
}

// 串口指令处理
void  UARTCmdProcess(void)
{
	// 班级
	if (strcmp((char*)uart_char_buffer, "\nAT+CLASS") == 0){
		UARTStringPutNonBlocking("CLASS 2241");
	}
	
	// 学号
	else if (strcmp((char*)uart_char_buffer, "\nAT+ID") == 0){
		UARTStringPutNonBlocking("ID 522021910121");
	}

	// 帮助
	else if (strcmp((char*)uart_char_buffer, "\nHELP") == 0){
		HelpSend();
	}

	// 恢复出厂设置
	else if (strcmp((char*)uart_char_buffer, "\nINIT CLOCK") == 0){
		BootScreen();
	}

	// 获得时间
	else if (strcmp((char*)uart_char_buffer, "\nGET TIME") == 0){
		TimeSend();
	}

	// 获得日期
	else if (strcmp((char*)uart_char_buffer, "\nGET DATE") == 0){
		DateSend();
	}

	// 获得闹钟
	else if (strcmp((char*)uart_char_buffer, "\nGET ALARM") == 0){
		AlarmSend();
	}

	// 播放音乐
	else if (strcmp((char*)uart_char_buffer, "\nMUSIC ON") == 0){
		SetMusicOn();
	}

	// 暂停音乐
	else if (strcmp((char*)uart_char_buffer, "\nMUSIC OFF") == 0){
		SetMusicOff();
	}

	// 设置时间
	else if (strncmp((char*)uart_char_buffer, "\nSET TIME", 9) == 0){
		h = (uart_char_buffer[10] - '0') * 10 + (uart_char_buffer[11] - '0');
		m = (uart_char_buffer[13] - '0') * 10 + (uart_char_buffer[14] - '0');
		s = (uart_char_buffer[16] - '0') * 10 + (uart_char_buffer[17] - '0');
		if (uart_char_buffer[12] != ':' && uart_char_buffer[15] != ':' || h >= 24 || m >= 60 || s >= 60){
			ErrorSend();
		}
		else{
			ClockGen(h, m, s);
			TimeSend();
		}
	}

	// 设置闹钟
	else if (strncmp((char*)uart_char_buffer, "\nSET ALARM TIME", 15) == 0){
		h = (uart_char_buffer[16] - '0') * 10 + (uart_char_buffer[17] - '0');
		m = (uart_char_buffer[19] - '0') * 10 + (uart_char_buffer[20] - '0');
		s = (uart_char_buffer[22] - '0') * 10 + (uart_char_buffer[23] - '0');
		if (uart_char_buffer[18] != ':' && uart_char_buffer[21] != ':' || h >= 24 || m >= 60 || s >= 60){
			ErrorSend();
		}
		else{
			AlarmGen(h, m, s);
			AlarmSend();
		}
	}

	// 设置日期
	else if (strncmp((char*)uart_char_buffer, "\nSET DATE", 9) == 0){
		y1 = (uart_char_buffer[10] - '0') * 1000 + (uart_char_buffer[11] - '0') * 100 + (uart_char_buffer[12] - '0') * 10 + (uart_char_buffer[13] - '0');
		m1 = (uart_char_buffer[15] - '0') * 10 + (uart_char_buffer[16] - '0');
		d1 = (uart_char_buffer[18] - '0') * 10 + (uart_char_buffer[19] - '0');
		if (uart_char_buffer[14] != '-' && uart_char_buffer[17] != '-' || m1 > 12 || d1 > 31){
			ErrorSend();
		}
		else{
			DateGen(y1, m1, d1);
			DateSend();
		}
	}

	else {
		ErrorSend();
	}
}

// 发送错误指令
void ErrorSend(void)
{
	UARTStringPutNonBlocking("ERROR!\n");
	UARTStringPut("Enter \'HELP\' to See CMDs...\n");

}

// 发送时间
void TimeSend(void)
{
	uint8_t  timeShowStr[20] = "TIME ";
	UARTStringPutNonBlocking(strcat((char*)timeShowStr, (const char *)timeStr));
}

// 发送日期
void DateSend(void)
{
	uint8_t  dateShowStr[20] = "DATE ";
	UARTStringPutNonBlocking(strcat((char*)dateShowStr, (const char *)dateStr));
}

// 发送闹钟
void AlarmSend(void)
{
	uint8_t  alarmShowStr[20] = "ALARM ";
	UARTStringPutNonBlocking(strcat((char*)alarmShowStr, (const char *)alarmStr));
}

// 发送提示信息
void HelpSend(void)
{
	CharEcho('\n', 1);
	CharEcho('-', 50);
	UARTStringPut("-*- THIS IS HELP INFOMATION -*-\n\n");
	UARTStringPut("01 \"INIT CLOCK\": Reset the Clock to the Initial State\n");
	UARTStringPut("02 \"GET TIME\": Acquire the Time\n");
	UARTStringPut("03 \"GET DATE\": Acquire the Date\n");
	UARTStringPut("04 \"GET ALARM\": Acquire the Alarm Time\n");
	UARTStringPut("05 \"SET TIME HH:MM:SS\": Set the Time to HH:MM:SS\n");
	UARTStringPut("06 \"SET DATE YYYY-MM-DD\": Set the Date to YYYY-MM-DD\n");
	UARTStringPut("07 \"SET ALARM TIME HH:MM:SS\": Set the Alarm Time to HH:MM:SS\n");
	UARTStringPut("08 \"AT+CLASS\": Acquire the Class Number\n");
	UARTStringPut("09 \"AT+ID\": Acquire the Student ID\n");
	UARTStringPut("10 \"MUSIC ON\": Set Music ON\n");
	UARTStringPut("11 \"MUSIC OFF\": Set Music OFF\n");
	CharEcho('-', 50);
}



/* 功能函数*/

// 音乐播放
void TIMER1A_Handler(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // 清除中断标志 
	
	// 处理一个新音符
	PWMGenDisable(PWM0_BASE, PWM_GEN_3); // PWM 信号暂停

	if (CurPos == SizeOfSong) {
		CurPos = 0;
	}
	else {
		// 设置音符的播放时间为 TIMER1 的定时时长
		TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock/1000.0 * time[CurPos] * BeatTime); 

		if (freqs[CurPos] != 0) {
			// 根据音频计算 PWM 信号的周期，并启动产生 PWM 信号 
			PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32SysClock / freqs[CurPos]);  // 设定周期
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 2); 	// 设置占空比等于 50%
			if (MusicOn) PWMGenEnable(PWM0_BASE, PWM_GEN_3); // 使能 PWM 信号产生
		} 
		CurPos++; // 向后推进
	}
}

void SysTick_Handler(void)
{
    if (++systick_1000ms_couter >= 1000)
	{
		systick_1000ms_couter = 0;
		systick_1000ms_status = 1;
	}

	if (++systick_500ms_couter >= 500)
	{
		systick_500ms_couter = 0;
		systick_500ms_status = 1;
	}

	if (++systick_250ms_couter >= 250)
	{
		systick_250ms_couter = 0;
		systick_250ms_status = 1;
	}

	if (++systick_10ms_couter >= 10)
	{
		systick_10ms_couter = 0;
		systick_10ms_status = 1;
	}

    if (++systick_1ms_couter >= 1)
	{
		systick_1ms_couter = 0;
		systick_1ms_status = 1;
	}

			
}

void UART0_Handler(void)
{
	int32_t uart0_int_status;
	volatile int8_t blankFlag = 0;
    uart0_int_status = UARTIntStatus(UART0_BASE, true);

	// 清除中断
	UARTIntClear(UART0_BASE, uart0_int_status);

	while(UARTCharsAvail(UART0_BASE)) 
	{
        uart_receive_char = UARTCharGetNonBlocking(UART0_BASE);
        if (uart_receive_char != '\r'){
			 // 全部转为大写字母
            if (uart_receive_char >= 'a' && uart_receive_char <= 'z'){
                uart_receive_char = uart_receive_char - 'a' + 'A';
            }

			// 清除冗余的空格
			if (uart_receive_char != ' '){
				blankFlag = 0;
				uart_char_buffer[uart_char_curIdx++] = uart_receive_char; // 写入Buffer数组 
			}
			else {
				if (!blankFlag) { // 第一次遇到空格
					blankFlag = 1;
					uart_char_buffer[uart_char_curIdx++] = uart_receive_char; // 写入Buffer数组
				}
			}
        }
        else {
            uart_receive_status = 1;
            uart_char_buffer[uart_char_curIdx] = '\0';
            uart_char_curIdx = 0;
        }			
	}
}

// 时钟生成
void ClockGen( uint8_t h, uint8_t m, uint8_t s)
{
	if (s >= 60){
		m += s / 60;
		s %= 60;
	}
	if (m >= 60){
		h += m / 60;
		m %= 60;
	}
	if (h >= 24){
		day += h / 24;
		h %= 24;
	}
	hh = h; mm = m; ss = s;
	timeStr[0] = digits1[0] = hh / 10 + '0';
	timeStr[1] = digits1[1] = hh % 10 + '0';
	timeStr[3] = digits1[3] = mm / 10 + '0';
	timeStr[4] = digits1[4] = mm % 10 + '0';
	timeStr[6] = digits1[6] = ss / 10 + '0';
	timeStr[7] = digits1[7] = ss % 10 + '0';
}

// 日期生成
void DateGen( uint16_t y, uint8_t m, uint8_t d)
{
	uint8_t DaysOfMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	if ((y % 100 != 0 && y % 4 == 0) || y % 400 == 0){ 	// 闰年
		DaysOfMonth[2] = 29;
	}

	if (d > DaysOfMonth[m]){
		m += d / DaysOfMonth[m];
		d %= DaysOfMonth[m - 1];
	}
	if (m > 12){
		y += m / 12;
		m %= 12;
	}


	year = y; mon = m; day = d;
	dateStr[0] = digits2[0] = year / 1000 + '0';
	dateStr[1] = digits2[1] = year / 100 % 10 + '0';
	dateStr[2] = digits2[2] = year / 10 % 10 + '0';
	dateStr[3] = digits2[3] = year % 10 + '0';
	dateStr[5] = digits2[4] = mon / 10 + '0';
	dateStr[6] = digits2[5] = mon % 10 + '0';
	dateStr[8] = digits2[6] = day / 10 + '0';
	dateStr[9] = digits2[7] = day % 10 + '0';
}

// 闹钟生成
void AlarmGen( uint8_t h, uint8_t m, uint8_t s)
{
	AlarmH = h;
	AlarmM = m;
	AlarmS = s;

	alarmStr[0] = AlarmH / 10 + '0';
	alarmStr[1] = AlarmH % 10 + '0';
	alarmStr[3] = AlarmM / 10 + '0';
	alarmStr[4] = AlarmM % 10 + '0';
	alarmStr[6] = AlarmS / 10 + '0';
	alarmStr[7] = AlarmS % 10 + '0';

}

// 按键检测与解码
void KeyDecoder(void)
{
	uint8_t tmp, i;
	tmp = ~I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0);
	key_value = 0;
	for (i = 1; i <= 8; i++){
		if ((1 << (i - 1)) & tmp) {
			key_value = i;
			break;
		}
	}

	// 蓝板上SW1~SW8检测
	switch (key_status){
		case 0:	// 前一次没按，这一次按下
			if (key_value != 0){
				key_status = 1;
				if (key_value == Key_Right){
					key_right = 1;
					if (key_mode != Key_Set){
						DisplayStatus = 4;
						key_mode = Key_Null;
					}
				}
				else if (key_value == Key_Left){
					key_left = 1;
					if (key_mode != Key_Set){
						DisplayStatus = 3;
						key_mode = Key_Null;
					}
				}
				else if (key_value == Key_Enter){
					key_enter = 1;
				}
				else if (key_value == Key_Music){
					SetMusicOn();
				}
				else {
					key_mode = key_value;
				}
			}
		case 1:	// 前一次按下，这一次没按
			if (key_value == 0)
			{
				key_status = 0;
			}	
			else {
				if (key_value == Key_Left && ++key_left_timer >= KEY_TIMER){
					key_left_timer = 0;
					DisplayStatus = 5;
					key_mode = Key_Null;
				}
				if (key_value == Key_Right && ++key_right_timer >= KEY_TIMER){
					key_right_timer = 0;
					DisplayStatus = 6;
					key_mode = Key_Null;
				}
			}	
	}

	// 红板上SW2检测
	switch (key_plus_status)
	{
		case 0:
			if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){ // 按下PJ1-SW2
				key_plus_status = 1;
				key_plus = 1;
			}
			break;
		case 1:
			if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) != 0){ // 未按下PJ1-SW2
				key_plus_status = 0;
			}
			break;
		default:
			break;
	}

	// 红板上SW1检测
	switch (key_minus_status)
	{
		case 0:
			if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){ // 按下PJ0-SW1
				key_minus_status = 1;
				key_minus = 1;
			}
			break;
		case 1:
			if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) != 0){ // 未按下PJ0-SW1
				key_minus_status = 0;
			}
			break;
		default:
			break;
	}

}

void DisplayModeOn(void)
{
	switch (DisplayStatus)
	{
	case 1: // 数码管刷新日期
		DigitWrite(digits2, 8);
		break;

	case 2: // 数码管刷新时间
		DigitWrite(digits1, 8); 
		break;

	case 3:	// 左流水，慢显示
		if (systick_500ms_status){
			systick_500ms_status = 0;
			RunningIdx = (RunningIdx + 1) % 18;
			LedPos = (LedPos == 0x01) ? 0x80 : LedPos;
			LedPos >>= 1;
		}
		DigitWrite(fullStr + RunningIdx, 8);
		LedLight(LedPos);
		break;

	case 4: // 右流水，慢显示
		if (systick_500ms_status){
			systick_500ms_status = 0;
			RunningIdx = (RunningIdx - 1 + 18) % 18;
			LedPos = (LedPos == 0x80) ? 0x01 : LedPos;
			LedPos <<= 1;
		}
		DigitWrite(fullStr + RunningIdx, 8);
		LedLight(LedPos);
		break;

	case 5: // 左流水，快显示
		if (systick_250ms_status){
			systick_250ms_status = 0;
			RunningIdx = (RunningIdx + 1) % 18;
			LedPos = (LedPos == 0x01) ? 0x80 : LedPos;
			LedPos >>= 1;
		}
		DigitWrite(fullStr + RunningIdx, 8); 
		LedLight(LedPos);
		break;

	case 6: // 右流水，快显示
		if (systick_250ms_status){
			systick_250ms_status = 0;
			RunningIdx = (RunningIdx - 1 + 18) % 18;
			LedPos = (LedPos == 0x80) ? 0x01 : LedPos;
			LedPos <<= 1;
		}
		DigitWrite(fullStr + RunningIdx, 8);
		LedLight(LedPos);
		break;

	default:
		break;
	}

}

// 设置时间、日期、闹钟
void  SetModeOn(void)
{
	// 闪烁D4
	if (systick_10ms_status)
		{
			systick_10ms_status	= 0;
			if (++ gpio_flash_cnt >= GPIO_FLASHTIME/10)
			{
				gpio_flash_cnt	= 0;
				if (gpio_status)
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_PIN_0 ); // 点亮PF0-D4
				else
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0);
				gpio_status	= !gpio_status;
			}
	}

	
	if (DisplayStatus == 1){	// 设置日期
		SetDate();
	}
	else if (DisplayStatus == 2) {	// 设置时间和闹钟
		SetTime();
		if (MusicOn){
			SetMusicOff();
			SetAlarm();
		}
	}

	// 确认时间或日期设置
	if (key_enter){
		key_enter = 0;
		if (DisplayStatus == 1){
			year = y1; mon = m1; day = d1;
			day = (day > 0) ? day : day + 1;
			mon = (mon > 0) ? mon : mon + 1;
		}
		else if (DisplayStatus == 2) {
			hh = h; mm = m; ss = s;	
		}
		key_mode = DisplayStatus;
	}

}

// 时间设置的具体实现
void SetTime(void)
{
	static uint8_t curPos = 0;	   // 当前处理的数字
	LedLight(0x1 << (7 - curPos)); // 亮起当前数字对应的LED
	h = hh; m = mm; s = ss;

	if (key_left){
		key_left = 0;
		curPos = (curPos + 1) % 8;
	}

	if (key_right){
		key_right = 0;
		curPos = (curPos - 1 + 8) % 8;
	}

	if (key_plus){
		key_plus = 0;
		switch (curPos)
		{
			case 0:
				s = (s + 1) % 60;
				break;
			case 1:
				s = (s + 10) % 60;
				break;
			case 3:
				m = (m + 1) % 60;
				break;
			case 4:
				m = (m + 10) % 60;
				break;
			case 6:
				h = (h + 1) % 24;
				break;
			case 7:
				h = (h + 10) % 24;
			default:
				break;
		}
	}

	if (key_minus){
		key_minus = 0;
		switch (curPos)
		{
			case 0:
				s = (s - 1 + 60) % 60;
				break;
			case 1:
				s = (s - 10 + 60) % 60;
				break;
			case 3:
				m = (m - 1 + 60) % 60;
				break;
			case 4:
				m = (m - 10 + 60) % 60;
				break;
			case 6:
				h = (h - 1 + 24) % 24;
				break;
			case 7:
				h = (h - 10 + 24) % 24;
			default:
				break;
		}
	}

	ClockGen(h, m, s);
}

// 日期设置的具体实现
void SetDate(void)
{
	static uint8_t curPos = 0;	   // 当前处理的数字
	LedLight(0x1 << (7 - curPos)); // 亮起当前数字对应的LED
	y1 = year; m1 = mon; d1 = day;

	if (key_left){
		key_left = 0;
		curPos = (curPos + 1) % 8;
	}

	if (key_right){
		key_right = 0;
		curPos = (curPos - 1 + 8) % 8;
	}

	if (key_plus){
		key_plus = 0;
		switch (curPos)
		{
			case 0:
				d1 = (d1 % 10 + 1) % 10 + d1 / 10 * 10;
				break;
			case 1:
				d1 = (d1 / 10 + 1) % 4 * 10 + d1 % 10;
				break;
			case 2:
				m1 = (m1 + 1) % 13;
				break;
			case 3:
				m1 = (m1 / 10 + 1) % 2 * 10 + m1 % 10;
				break;
			case 4:
				y1 = (y1 + 1) % 10000;
				break;
			case 5:
				y1 = (y1 + 10) % 10000;
				break;
			case 6:
				y1 = (y1 + 100) % 10000;
			case 7:
				y1 = (y1 + 1000) % 10000;
			default:
				break;
		}
	}

	if (key_minus){
		key_minus = 0;
		switch (curPos)
		{
			case 0:
				d1 = (d1 % 10 - 1 + 10) % 10 + d1 / 10 * 10;
				break;
			case 1:
				d1 = (d1 / 10 + 4 - 1) % 4 * 10 + d1 % 10;
				break;
			case 2:
				m1 = (m1 - 1 + 13) % 13;
				break;
			case 3:
				m1 = (m1 / 10 + 2 - 1) % 2 * 10 + m1 % 10;
				break;
			case 4:
				y1 = (y1 - 1 + 10000) % 10000;
				break;
			case 5:
				y1 = (y1 - 10 + 10000) % 10000;
				break;
			case 6:
				y1 = (y1 - 100 + 10000) % 10000;
			case 7:
				y1 = (y1 - 1000 + 10000) % 10000;
			default:
				break;
		}
	}

	DateGen(y1, m1, d1);
}

// 闹钟设置的具体实现
void SetAlarm(void)
{
	AlarmGen(h, m, s);
}

// 从头播放音乐
void SetMusicOn(void) 
{
	MusicOn = 1;
	CurPos = 0;
}

// 暂停播放音乐
void SetMusicOff(void)
{
	MusicOn = 0;
}

// 根据按键选择模式
void  ModeSelect(void)
{
	switch (key_mode)
	{
		case Key_Return:	// 一键返回
			key_mode = Key_Null;
			key_right = key_left = key_enter = key_plus = key_minus= 0;
			SetMusicOff();
			break;
		case Key_Date:		// 显示日期
			DisplayStatus = 1;
			LedLight(0x1 << 0);
			break;
		case Key_Time:		// 显示时间
			DisplayStatus = 2;
			LedLight(0x1 << 1);
			break;
		case Key_Set:		// 设置日期或时间
			SetModeOn();
			break;
		
		default:
			break;
	}
}
