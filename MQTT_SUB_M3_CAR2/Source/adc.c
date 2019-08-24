#include "device_driver.h"

extern int      ADC1_value;
extern int      ADC1_flag;

void ADC1_Init(void)
{
    // CLK Enable
    Macro_Set_Bit(RCC->APB2ENR,9);              // ADC_CLK Enable - ADC1 Enable
        
    // Register Configuration
    // AWD disable on regular channel[23] & injected channel[22], about dual - Indepent mode[16]
    ADC1->CR1   = (0<<23)|(0<<22)|(0x0<<16);
    ADC1->CR2   = (0<<11)|(0<<1);              // Right alignment mode[11], Single conversion mode[1]

    // Sample time of all channel - 239.5 cycles
    Macro_Write_Block(ADC1->SMPR1,0xffffffff,0x00ffffff,0);
    Macro_Write_Block(ADC1->SMPR2,0xffffffff,0x3fffffff,0);
}

int ADC1_Get_Data(int ch)
{
    int value = 0;
    int i;
    
    // Pin Configuration
    if(ch<8)
    {
        Macro_Set_Bit(RCC->APB2ENR,2);
        Macro_Write_Block(GPIOA->CRL,0xf,0x0,ch*4);
    }
    else
    {
        Macro_Set_Bit(RCC->APB2ENR,3);
        Macro_Write_Block(GPIOB->CRL,0xf,0x0,(ch-8)*4);
    }

    Macro_Write_Block(ADC1->SQR1,0xf,0x0,20);           // Regular channel sequence length : 1 (1 conversion)
    Macro_Write_Block(ADC1->SQR3,0x1f,(ch & 0x1f),0);   // SQR0 - ADC_IN0
    Macro_Clear_Bit(ADC1->CR2,1);                       // single operation mode
    
    Macro_Set_Bit(ADC1->CR2,0);                         // Wake-up from power-down mode
    while(!(ADC1->CR2 & 0x1));                          // wait wake-up
    for(i=0;i<0xffff;i++);
    Macro_Set_Bit(ADC1->CR2,0);                         // Start Conversion
    while(!(ADC1->SR & (1<<1)));                        // wait end of conversion (this bit will cleared by reading ADC1_DR)
    value = ADC1->DR;
    Macro_Clear_Bit(ADC1->CR2,0);                       // Stop
    
    return value;    
}

void ADC1_ISR_Enable(int en,int ch)
{
    NVIC_ClearPendingIRQ((IRQn_Type)18);
    if(en)
    {
        Macro_Set_Bit(ADC1->CR1,5);
	    NVIC_EnableIRQ((IRQn_Type)18);
        if(ch<8)
        {
            Macro_Set_Bit(RCC->APB2ENR,2);
            Macro_Write_Block(GPIOA->CRL,0xf,0x0,ch*4);
        }
        else
        {
            Macro_Set_Bit(RCC->APB2ENR,3);
            Macro_Write_Block(GPIOB->CRL,0xf,0x0,(ch-8)*4);
        }
        Macro_Write_Block(ADC1->SQR1,0xf,0x0,20);           // Regular channel sequence length : 1 (1 conversion)
        Macro_Write_Block(ADC1->SQR3,0x1f,(ch & 0x1f),0);   // SQR0 - ADC_IN0
        Macro_Clear_Bit(ADC1->CR2,1);                       // single operation mode
    }
    else
    {
        Macro_Clear_Bit(ADC1->CR1,5);
        NVIC_DisableIRQ((IRQn_Type)18);

    }
}

void ADC1_ISR_Start(void)
{
    int i;

    Macro_Set_Bit(ADC1->CR2,0);                         // Wake-up from power-down mode
    while(!(ADC1->CR2 & 0x1));                          // wait wake-up
    for(i=0;i<0xffff;i++);
    Macro_Set_Bit(ADC1->CR2,0);                         // Start Conversion      
}

void ADC1_ISR_Stop(void)
{
    Macro_Clear_Bit(ADC1->CR2,0);                       // Stop
}
