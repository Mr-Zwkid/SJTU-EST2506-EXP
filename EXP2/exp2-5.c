
#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"

//I2C GPIO chip address and resigster define

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

#define FastFlashTime 800000
#define SlowFlashTime 6000000

#define SysTickFrequency 1000 		// Systick 频率为 1000Hz，周期为 1ms

void 		Delay(uint32_t value);
void 		Device_Init(void);
void 		SysTick_Init(void);
void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
void 		DigitWrite( uint8_t *data, uint8_t n);		// 向数码管中依次写入数字

uint32_t 	ui32SysClock;
uint8_t 	seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
volatile uint8_t 	result;

// 计时相关变量
volatile uint16_t   systick_1ms_couter = 0, systick_200ms_couter = 0, systick_1000ms_couter = 0;
volatile uint8_t	systick_1ms_status, systick_200ms_status, systick_1000ms_status;

uint8_t digits[] = {1, 2, 0, 0, ' ', ' ', ' ', ' '};
uint16_t sec = 00;
uint16_t min = 12;
uint8_t SW_state;	// 按键状态，1 为有按键按下、0 为无按键按下
uint8_t SW_value;   // 按键值

int main(void)
{
	// use internal 16M oscillator, HSI
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	
	Device_Init();
	
	while (1)
	{
		if (systick_1000ms_status){

			systick_1000ms_status = 0;
			sec++;
		}

		if (systick_1ms_status){
			systick_1ms_status = 0;
			if (sec >= 60){
				sec -= 60;
				min++;
			}
			if (min >= 60){
				min -= 60;
			}
			digits[0] = min / 10;
			digits[1] = min % 10;
			digits[2] = sec / 10;
			digits[3] = sec % 10;
			DigitWrite(digits, 8);
		}

	}
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

// GPIO初始化
void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);				//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);				//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			         //Set PF0 as Output pin
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);       //Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

// I2C初始化
void S800_I2C0_Init(void)
{
	uint8_t result;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);			// 初始化I2C模块
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);		// 使用I2C模块0，引脚配置为I2C0SCL-PB2、I2C0SDA-PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);					// 配置PB2为I2C0SCL
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);					// 配置PB3为I2C0SDA
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);		// I2C将GPIO_PIN_2用作SCL
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);		// I2C将GPIO_PIN_3用作SDA

	I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true);  // config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);				//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){}; // 如果I2C0模块忙，等待

	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false); // 设置主机要放到总线上的从机地址。false表示主机写从机，true表示主机读从机
	I2CMasterDataPut(I2C0_BASE, RegAddr);  // 主机写设备寄存器地址
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 执行重复写入操作
	while(I2CMasterBusy(I2C0_BASE)){};
		
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//调试用

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);//执行重复写入操作并结束
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//调试用

	return rop;  // 返回错误类型，无错返回0
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	// I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);//执行单词写入操作
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//设置从机地址
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//执行单次读操作
	while(I2CMasterBusBusy(I2C0_BASE));
	value = I2CMasterDataGet(I2C0_BASE);//获取读取的数据
	Delay(1);
	return value;
}

void SysTick_Init(void){
	SysTickPeriodSet(ui32SysClock/SysTickFrequency);
	SysTickEnable();
	SysTickIntEnable();
    IntMasterEnable();
}


void SysTick_Handler(void)
{
	if (++systick_200ms_couter >= 200)
	{
		systick_200ms_couter = 0;
		systick_200ms_status = 1;
	}
	
	if (++systick_1ms_couter >= 1)
	{
		systick_1ms_couter	= 0;
		systick_1ms_status = 1;
	}	
	if (++systick_1000ms_couter >= 1000)
	{
		systick_1000ms_couter = 0;
		systick_1000ms_status = 1;
	}	

	SW_value = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	switch (SW_state)
	{
	case 1: // 前一次有键按下
		if (SW_value == 3){
			SW_state = 0;
		}
		else {
			if (systick_200ms_status){
				systick_200ms_status = 0;
				sec++;
			}
		}
		break;
	case 0: // 前一次无键按下
		if (SW_value != 3){
			SW_state = 1;
			sec++;
		}
		break;
	
	default:
		break;
	}


}

void Device_Init(void){

	S800_GPIO_Init();
	S800_I2C0_Init();
	SysTick_Init();

}

void DigitWrite( uint8_t *data, uint8_t n){
	uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
	uint8_t lsh = 0x1;
	uint8_t i;
	for (i = 0; i < n; i++){
		if (data[i] <= 9){
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1, seg7[data[i]]);	
			// 显示的位置，高有效								
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, lsh);
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, 0x0);

			lsh <<= 1;
			if (lsh == 0x80) lsh = 0x01;
		}
	}
}


