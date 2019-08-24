#include "device_driver.h"

void LED_Init(void)
{
  Peri_Set_Bit(RCC->APB2ENR, 3);
  Macro_Write_Block(GPIOB->CRH, 0xf, 0x6, 0);
  GPIO_Lock_Multi(B, 0x1<<8);
  GPIO_Set_Bit(B, 8);
}

void LED_Display(unsigned int num)
{
	if(num & 0x1)	GPIO_Clear_Bit(B, 8);
	else				GPIO_Set_Bit(B, 8);
}

void LED_On(void)
{
  GPIO_Clear_Bit(B, 8);
}

void LED_Off(void)
{
  GPIO_Set_Bit(B, 8);
}
