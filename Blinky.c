/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2015 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>

#include "Board_LED.h"                  // ::Board Support:LED
#include "Board_Buttons.h"              // ::Board Support:Buttons

#include "stm32f10x.h"                  // Device header

#include "Serial.h"


volatile uint32_t msTicks;                                 // counts 1ms timeTicks
///*----------------------------------------------------------------------------
// * SysTick_Handler:
// *----------------------------------------------------------------------------*/
//void SysTick_Handler(void) {
//  msTicks++;
//}

///*----------------------------------------------------------------------------
// * Delay: delays a number of Systicks
// *----------------------------------------------------------------------------*/
//void Delay (uint32_t dlyTicks) {
//  uint32_t curTicks;

//  curTicks = msTicks;
//  while ((msTicks - curTicks) < dlyTicks) { __NOP(); }
//}

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
                             (HSE is not populated on Nucleo board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {

  RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

  FLASH->ACR  = FLASH_ACR_PRFTBE;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_LATENCY;                         // Flash 1 wait state

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;                        // APB1 = HCLK/2
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 = HCLK

  RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

  //  PLL configuration:  = HSI/2 * 12 = 48 MHz
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
  RCC->CFGR |=  (RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL12);

  RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

  RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
}


int32_t LED_Initialize_1 (void) {

  RCC->APB2ENR |= (3UL << 2);                /* Enable GPIOA clock            */

  /* Configure LED (PA.5) pins as output */
  GPIOA->CRH   &= ~((15ul << 4*7));
  GPIOA->CRH   |=  (( 1ul << 4*7));
	
	GPIOA->CRH   &= ~((15ul << 4*3));
  GPIOA->CRH   |=  (( 1ul << 4*3));

  GPIOA->CRH   &= ~((15ul << 4*4));
  GPIOA->CRH   |=  (( 1ul << 4*4));
	
	GPIOA->CRL   &= ~((15ul << 0));
  GPIOA->CRL   |=  (( 1ul << 0));
	
	GPIOB->CRL   &= ~((15ul << 4));
  GPIOB->CRL   |=  (( 1ul << 4));
	
	GPIOB->CRH   &= ~((15ul << (13-8)*4));
  GPIOB->CRH   |=  (( 1ul << (13-8)*4));
	
	GPIOB->CRH   &= ~((15ul << (14-8)*4));
  GPIOB->CRH   |=  (( 1ul << (14-8)*4));
	
	GPIOB->CRH   &= ~((15ul << (15-8)*4));
  GPIOB->CRH   |=  (( 1ul << (15-8)*4));

  GPIOA->CRH   &= ~((15ul << 4*0));
  GPIOA->CRH   |=  (( 1ul << 4*0));
	
  return (0);
}

int32_t LED_On_1 (uint32_t num) {
  int32_t retCode = 0;
  GPIOA->BSRR = (1<<num);
  return retCode;
}

int32_t LED_Off_1 (uint32_t num) {
  int32_t retCode = 0;
  GPIOA->BSRR = (1<<(num+16));
  return retCode;
}


//void SPI1_Master_Init(void) {
////RCC->AHB1ENR |= 1;                  //Enable GPIOA clock
//RCC->APB2ENR |= 0x1000;        // Enable SPI1 clock

///*———————PORTA 5, 7 for SPI1 MOSI and SCLK———————–*/
//  GPIOA->CRL   &= ~(0xFFFFul <<  4*4);         /* clear PA2, PA3                  */
//  GPIOA->CRL   |=  (0x0Bul <<  4*4);         /* USART2 Tx (PA2) output push-pull*/
//  GPIOA->CRL   |=  (0x0Bul << 4*5);         /* USART2 Rx (PA3) ioutput push-pull  */                       // Set alt mode SPI1
//  GPIOA->CRL   |=  (0x0Bul <<  4*7);         /* USART2 Tx (PA2) output push-pull*/
//  GPIOA->CRL   |=  (0x04ul << 4*6);         /* USART2 Rx (PA3) input floating  */                       // Set alt mode SPI1
//SPI1->CR1 = 0x31C;                                        //Set the Baud rate, 8-bit data frame
//SPI1->CR2 = 0;
//SPI1->CR1 |= 0x40;                                         //Enable SPI1 module
//}

CS_SELECT(unsigned char a)
{
	if(a)
    GPIOA->BSRR = (1<<4);	
	else
		GPIOA->BSRR = (1<<(4+16));
}

char buff1[2][512], buff2[512];
int write_len=0,read_len=0, write_block_len = 0,state=0,stop_gps = 0,pos = 0,pos1 = 0;

void USART3_IRQHandler()
{
		if (USART3->SR & 0x0020)
		{
			int data = USART3->DR;
			//SER_PutChar(data);
			if(!stop_gps)
			{
				USART2->DR = (data);
				buff1[pos][write_len++] = data;
			}
		}
}



/*----------------------------------------------------------------------------
 * main: blink LED and check button state
 *----------------------------------------------------------------------------*/
char gps_cmd[] = {"$PSTMSETPAR,1228,8000*1A\n"};
char gps_cmd2[] = {"$PSTMSAVEPAR*58\n"};


int main (void) {
  int32_t max_num = LED_GetCount();
  int32_t num = 0;
  int i = 0,j,test[20];
	
	int a,b;
	DelayInit();
  SystemCoreClockConfigure();                              // configure HSI as System Clock
  SystemCoreClockUpdate();
RCC->APB2ENR |= (3UL << 2);                /* Enable GPIOA clock            */
  
  //Buttons_Initialize();
  SER_Initialize();
	SD_Init();
	SD_GetCapacity();
	
	a = SD_GetCID(&b);
	
	//a = SD_WriteSingleBlock( 0,buff1);
	//printf("Write buffer status : %d", a);
	//a = SD_ReadSingleBlock( 0,buff2);
	//printf("read buffer status : %d and data : %s", a, buff2);
	
		
	//LED_Initialize_1();
	//SPI1_Master_Init();
  GPIOB->CRL   &= ~((15ul << 4*0));
  GPIOB->CRL   |=  (( 1ul << 4*0));
//  SysTick_Config(SystemCoreClock / 1000);                  // SysTick 1 msec interrupts
  //GPIOB->BSRR = (1<<1);		// Turn specified LED on
	//GPIOB->BSRR = (1<<0);		// Turn specified LED on
	//GPIOB->BSRR = (1<<(14+16));		// Turn specified LED on
	//GPIOB->BSRR = (1<<15);		// Turn specified LED on
	GPIOB->BSRR = (1<<0);		// Turn specified LED on
	//GPIOB->BSRR = (1<<13);
	//Delay(50000);
	
	for(j=0;j<sizeof(gps_cmd);j++)
	{
			while (!(USART3->SR & 0x0080));
				USART3->DR = (gps_cmd[j] & 0xFF);
	}
		for(j=0;j<sizeof(gps_cmd2);j++)
	{
			while (!(USART3->SR & 0x0080));
				USART3->DR = (gps_cmd2[j] & 0xFF);
	}
  for (;;) {
    
			  if (USART2->SR & 0x0020)
				{
					
					int data = USART2->DR;
					if(data == 'S')
					{
						state = 1;
					}
					else if((state == 1) && (data == 'E'))
					{
						 state = 2;
					}
					else if((state == 2) && (data == 'N'))
					{
						 state = 3;
					}
					else if((state == 3 ) && (data == 'D'))
					{
						i = 0;
						stop_gps = 1;
						while(write_block_len)
						{
								i++;
								a = SD_ReadSingleBlock( i,buff2);
								//printf("read buffer status : %d and data : %s", a, buff2);
								write_block_len--;
							  for(j=0;j<512;j++)
								{
									  while (!(USART2->SR & 0x0080));
											USART2->DR = (buff2[j] & 0xFF);
								}
						}
						stop_gps = 0;
					}
				}
//					//SER_PutChar(data);
//					USART2->DR = (data);
//					//SPI1->DR = 0x31;
//					buff1[write_len++] = data;
					if(write_len >= 512 )
					{
						write_len = 0;
						pos1 = pos;
						if(pos)
						{
							pos = 0;
						}
						else
						{
							pos = 1;
						}
						a = SD_WriteSingleBlock( write_block_len++,buff1[pos1]);
						printf("Write buffer status : %d, block length: %d\n", a,write_block_len);
						write_len = 0;
					}
//				}

		}
			
	
}
