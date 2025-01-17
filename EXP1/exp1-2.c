// #include <stdint.h>
// #include <stdbool.h>
// #include "hw_memmap.h"
// #include "debug.h"
// #include "gpio.h"
// #include "hw_types.h"
// #include "pin_map.h"
// #include "sysctl.h"


// #define   FASTFLASHTIME			(uint32_t) 300000
// #define   SLOWFLASHTIME			(uint32_t) FASTFLASHTIME*20


// void 		Delay(uint32_t value);
// void 		S800_GPIO_Init(void);
// void		PF0_Flash(uint32_t key_value);

// uint32_t ui32SysClock;
// uint32_t read_key_value;

// int main(void)
// {
// 	ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_INT|SYSCTL_USE_OSC), 16000000);
// 	S800_GPIO_Init();
// 	while(1)
//     {
// 		read_key_value = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1);
// 		PF0_Flash(read_key_value);
//     }
// }

// void PF0_Flash(uint32_t key_value)
// {
// 	switch (key_value)
// 	{
// 	case 0: // 都按
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);			// Turn on the LED.
// 		break;
// 	case 1: // 按 switch2，亮 LED1
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
// 		break;
// 	case 2: // 按 switch1，亮 LED0
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);
// 		break;
// 	case 3: // 都没按
// 		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x0);	
// 		break;
// 	default:
// 		break;
// 	}
// }

// void Delay(uint32_t value)
// {
// 	uint32_t ui32Loop;
// 	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
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
// }


