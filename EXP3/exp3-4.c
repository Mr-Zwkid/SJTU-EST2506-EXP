
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

#define SYSTICK_FREQUENCY		1000			//1000hz

#define	I2C_FLASHTIME			500				//500mS
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

void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
void 		Device_Init(void);
void        UARTStringPutNonBlocking(const char *cMessage);
void        UARTStringPut(const char *cMessage);
void        DigitWrite( uint8_t *data, uint8_t n);
void        ClockGen( uint8_t h, uint8_t m, uint8_t s);
void		ErrorCmd(void);
void		TimeSend(void);

// Systick软定时
volatile uint16_t   systick_1ms_couter ,systick_10ms_couter = 0, systick_100ms_couter = 0, systick_1000ms_couter = 0;
volatile uint8_t	systick_1ms_status ,systick_10ms_status, systick_100ms_status, systick_1000ms_status;

volatile uint8_t result, cnt, key_value, gpio_status;
uint32_t ui32SysClock;
uint8_t  seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c, 0x40};

// UART 相关
uint8_t  uart_receive_char;
uint8_t  uart_receive_status;
uint8_t  uart_char_buffer[200];
uint8_t  uart_char_curIdx = 0;

// 计时有关变量
uint8_t  hh = 12;      // 全局小时数
uint8_t  mm = 00;      // 全局分钟数
uint8_t  ss = 00;      // 全局秒数
uint8_t  h, m, s;     // 命令处理中暂存时间的变量
uint8_t  digits[] = {'1', '2', '-', '0', '0', '-', '0', '0'}; // 数码管显示数字
uint8_t  timeStr[] = "12:00:00";  // 当前时间的字符串

// 月份
uint8_t * Month[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};

int main(void)
{
	volatile uint16_t	i2c_flash_cnt, gpio_flash_cnt;
	uint8_t i, num;	
	uint8_t m1, s1, m2, s2;
	unsigned char ms[] = "12:00";

	Device_Init();
	
	while (1)
	{
		// 按下SW2
		while (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == 0){
			IntTrigger(INT_UART0);
		}

		// 指令字符串处理
        if (uart_receive_status)
        {
            uart_receive_status = 0;

			// 班级
            if (strcmp((char*)uart_char_buffer, "\nAT+CLASS") == 0){
                UARTStringPutNonBlocking("CLASS2241");
            }
            
			// 学号
			if (strcmp((char*)uart_char_buffer, "\nAT+STUDENTCODE") == 0){
                UARTStringPutNonBlocking("CODE522021910121");
            }

			// 获得时间
			if (strcmp((char*)uart_char_buffer, "\nGETTIME") == 0){
				TimeSend();
			}

			// 设置或增加时间
			if (strncmp((char*)uart_char_buffer, "\nSET", 4) == 0  || strncmp((char*)uart_char_buffer, "\nINC", 4) == 0){
				h = (uart_char_buffer[4] - '0') * 10 + (uart_char_buffer[5] - '0');
				m = (uart_char_buffer[7] - '0') * 10 + (uart_char_buffer[8] - '0');
				s = (uart_char_buffer[10] - '0') * 10 + (uart_char_buffer[11] - '0');
				if (uart_char_buffer[6] != ':' && uart_char_buffer[9] != ':' || h >= 24 || m >= 60 || s >= 60){
					ErrorCmd();
				}
				else{
					if (uart_char_buffer[1] == 'S')
						ClockGen(h, m, s);
					else 
						ClockGen(h + hh, m + mm, s + ss);
					TimeSend();
				}
			}

			// 月份加减
			for (i = 0; i < 12; i++){
				if (strncmp((char*)(uart_char_buffer + 1), Month[i], 3) == 0){
					num = (uart_char_buffer[5] - '0') * 10 + (uart_char_buffer[6] - '0');
					if (num <= 11){
						if (uart_char_buffer[4] == '+') UARTStringPutNonBlocking(Month[(num + i) % 12]);
						else if (uart_char_buffer[4] == '-') UARTStringPutNonBlocking(Month[(i - num + 12) % 12]);
					}
					else ErrorCmd();
				}
			}

			if (strlen((const char*)uart_char_buffer) == 12 && uart_char_buffer[3] == ':' && uart_char_buffer[9] == ':'){
				m1 = (uart_char_buffer[1] - '0')* 10 + (uart_char_buffer[2] - '0');
				s1 = (uart_char_buffer[4] - '0')* 10 + (uart_char_buffer[5] - '0');
				m2 = (uart_char_buffer[7] - '0')* 10 + (uart_char_buffer[8] - '0');
				s2 = (uart_char_buffer[10] - '0')* 10 + (uart_char_buffer[11] - '0');

				if (m1 < 60 && s1 < 60 && m2 < 60 && s2 < 60){
					if (uart_char_buffer[6] == '+'){
						s1 += s2;
						m1 += m2;
						if (s1 >= 60){
							s1 -= 60;
							m1++;
						}
						if (m1 >= 60){
							m1 -= 60;
						}
					}
					else if (uart_char_buffer[6] == '-'){
						if (s1 < s2){
							s1 = s1 + 60 - s2;
							m2++;
						}
						else {
							s1 -= s2;
						}
						if (m1 < m2){
							m1 = m1 + 60 - m2;
						}
						else {
							m1 -= m2;
						}
					}

					ms[0] = m1 / 10 + '0';
					ms[1] = m1 % 10 + '0';
					ms[3] = s1 / 10 + '0';
					ms[4] = s1 % 10 + '0';

					UARTStringPutNonBlocking(ms);
				}
				else ErrorCmd();

			}

        }

        if (systick_1ms_status)
        {
            systick_1ms_status = 0;
            DigitWrite(digits, 8);
        }

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
		if (systick_1000ms_status)
		{
			systick_1000ms_status = 0;
			ClockGen(hh, mm, ss + 1);
		}
	}
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}


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
	UARTStringPut((const char *)"\r\nHello, world!\r\n");
}

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

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);				//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;

	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value, rop;

	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	// I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);

	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);

	// receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);

	while(I2CMasterBusBusy(I2C0_BASE));
	value = I2CMasterDataGet(I2C0_BASE);
	Delay(1);

	return value;
}

// 在 startup_TM4C129.s 中已提前定义中断服务函数
void SysTick_Handler(void)
{
    if (++systick_1000ms_couter >= 1000)
	{
		systick_1000ms_couter = 0;
		systick_1000ms_status = 1;
	}

	if (++systick_100ms_couter >= 100)
	{
		systick_100ms_couter = 0;
		systick_100ms_status = 1;
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

	if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0) // 按下SW1
	{
		systick_100ms_status = systick_10ms_status = 0;
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0); // 点亮PN0-D2
	}
	else
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,0);	
			
}

void UART0_Handler(void)
{
	int32_t uart0_int_status;
    uart0_int_status = UARTIntStatus(UART0_BASE, true);

	// 清除中断
	UARTIntClear(UART0_BASE, uart0_int_status);


	while(UARTCharsAvail(UART0_BASE)) 
	{
        uart_receive_char = UARTCharGetNonBlocking(UART0_BASE);
        if (uart_receive_char != '\r'){

            if (uart_receive_char >= 'a' && uart_receive_char <= 'z'){
                uart_receive_char = uart_receive_char - 'a' + 'A';
            } // 全部转为大写字母

            uart_char_buffer[uart_char_curIdx++] = uart_receive_char; // 写入Buffer数组
        }
        else {
            uart_receive_status = 1;
            uart_char_buffer[uart_char_curIdx] = '\0';
            uart_char_curIdx = 0;
        }			
	}
}

void Device_Init(void)
{
	// 系统时钟设置
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 20000000);

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();

	// Systick 初始化
	SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);
	SysTickEnable();
	SysTickIntEnable();

	// UART0 中断允许
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

	// 总中断允许
    IntMasterEnable();	
	
	// 中断优先级设置
	IntPriorityGroupingSet(3);		// 设置高3位全部用于抢占优先级设置，越小越优先
	IntPrioritySet(INT_UART0,0x0e0);	
	IntPrioritySet(FAULT_SYSTICK,0x03);
	
}

// 数码管写入数字
void DigitWrite( uint8_t *data, uint8_t n){
	uint8_t i;
    uint8_t tmp;
	for (i = 0; i < n; i++){
		if ((data[i] - '0' >= 0 && data[i] - '0' <= 9) || data[i] == '-') {
			tmp = (data[i] != '-') ? data[i] - '0' : 17;
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1, seg7[tmp]);	
			// 显示的位置，高有效								
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, 0x1 << i);
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2, 0x0);
		}	
	}
}

// 时钟生成
void ClockGen( uint8_t h, uint8_t m, uint8_t s)
{
	if (s >= 60){
		s -= 60;
		m++;
	}
	if (m >= 60){
		m -= 60;
		h++;
	}
	if (h >= 24){
		h -= 24;
	}
	hh = h; mm = m; ss = s;
	timeStr[0] = digits[0] = hh / 10 + '0';
	timeStr[1] = digits[1] = hh % 10 + '0';
	timeStr[3] = digits[3] = mm / 10 + '0';
	timeStr[4] = digits[4] = mm % 10 + '0';
	timeStr[6] = digits[6] = ss / 10 + '0';
	timeStr[7] = digits[7] = ss % 10 + '0';
}

// 错误指令
void ErrorCmd(void)
{
	UARTStringPutNonBlocking("ERROR!");
}

// 发送时间
void TimeSend(void)
{
	uint8_t  timeShowStr[20] = "TIME";
	UARTStringPutNonBlocking(strcat((char*)timeShowStr, (const char *)timeStr));
}
