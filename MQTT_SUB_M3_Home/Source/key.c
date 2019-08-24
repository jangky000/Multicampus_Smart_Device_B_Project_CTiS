#include "stm32f10x.h"
#include "macro.h"

void Key_Poll_Init(void)
{
  Peri_Set_Bit(RCC->APB2ENR, 2);
  Macro_Write_Block(GPIOA->CRH, 0xfff, 0x444, 20);
}

int Key_Get_Pressed(void)
{
  return ~GPIO_Extract_Area(A,0x7,13) & 0x7;
}

void Key_Wait_Key_Released(void)
{
  while(Key_Get_Pressed());
}

int Key_Wait_Key_Pressed(void)
{
  int k;

  do
  {
    k = Key_Get_Pressed();
  }while(!k);

  return k;
}

void Key_ISR_Enable(int en)
{
  if(en)
  {
		Macro_Set_Bit(RCC->APB2ENR,2);
		Macro_Set_Bit(RCC->APB2ENR,0);
	
		Macro_Write_Block(GPIOA->CRH,0xfff,0x444,20);
		Macro_Write_Block(AFIO->EXTICR[3],0xfff,0x000,4);
		Macro_Write_Block(EXTI->FTSR,0x7,0x7,13);

		EXTI->PR = (0x7<<13);
		NVIC_ClearPendingIRQ((IRQn_Type)40);
	  Macro_Write_Block(EXTI->IMR,0x7,0x7,13);
    NVIC_EnableIRQ((IRQn_Type)40);
  }
  else
  {
    NVIC_DisableIRQ((IRQn_Type)40);
    Macro_Clear_Bit(RCC->APB2ENR,2);
    Macro_Clear_Bit(RCC->APB2ENR,0);
  }
}
