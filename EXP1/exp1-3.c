// #include <stdint.h>
// #include <stdbool.h>
// #include "inc/hw_memmap.h"
// #include "driverlib/debug.h"
// #include "driverlib/gpio.h"
// #include "inc/hw_types.h"
// #include "driverlib/pin_map.h"
// #include "driverlib/sysctl.h"
// #include "driverlib/systick.h"    // SysTick Driver 原型
// #include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver 原型


// #define   FASTFLASHTIME			(uint32_t) 300000
// #define   SLOWFLASHTIME			(uint32_t) FASTFLASHTIME*20


// void 		Delay(uint32_t value);
// void 		S800_GPIO_Init(void);
// void		PF_Flash(uint32_t key_value);
// void 		SysTick_Handler(void);

// uint32_t ui32SysClock;
// uint32_t read_key_value;
// uint8_t cnt = 0;
// uint8_t key_state = 1;

// int main(void)
// {
// 	ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_INT|SYSCTL_USE_OSC), 16000000);
// 	S800_GPIO_Init();
// 	while(1)
//     {
// 		PF_Flash(cnt);
//     }
// }

// void PF_Flash(uint32_t key_value)
// {
// 	uint32_t delay_time = FASTFLASHTIME;
// 	switch (key_value)
// 	{
// 	case 1: // 第一次短按，闪烁 LED0
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
// 		Delay(delay_time);
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);	
// 		Delay(delay_time);
// 		break;
// 	case 2: // 第二次短按，熄灭 LED0
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);
// 		break;
// 	case 3: // 第三次短按，闪烁 LED1
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
// 		Delay(delay_time);
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);	
// 		Delay(delay_time);
// 		break;
// 	case 0: // 第四次短按，熄灭LED1
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);	
// 		break;
// 	default:
// 		break;
// 	}
// }

// void Delay(uint32_t value)
// {
// 	uint32_t ui32Loop;
// 	uint8_t pre_state = key_state;
// 	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){
// 		if (key_state != pre_state) break;
// 	};
// }


// void S800_GPIO_Init(void)
// {
// 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						
// 	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));		
	
// 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);	
// 	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			
	
//     GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);			
// 	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);
	
// 	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

// 	SysTickPeriodSet(ui32SysClock/50); // 设置心跳节拍,定时周期20ms
// 	SysTickEnable();  			// SysTick使能
// 	SysTickIntEnable();	
//     IntMasterEnable();
// }

// void SysTick_Handler(void)       // 定时周期为20ms
// {
// 	read_key_value = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0);
// 	switch (key_state){
// 		case 1: // 没被按
// 			if (read_key_value == 0){
// 				key_state = 0;
// 				cnt = (cnt + 1) % 4;
// 			}
// 			break;
// 		case 0: // 被按
// 			if (read_key_value == 1){
// 				key_state = 1;
// 			}
// 			break;
// 	}
// }

