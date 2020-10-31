/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low Level Serial Routines
 * Note(s): possible defines select the used communication interface:
 *            __DBG_ITM   - ITM SWO interface
 *                        - USART2 interface  (default)
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2014 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "stm32f10x.h"                  // Device header
#include "Serial.h"

#ifdef __DBG_ITM
volatile int ITM_RxBuffer;              /*  CMSIS Debug Input                 */
#endif

/*----------------------------------------------------------------------------
 Define  USART
 *----------------------------------------------------------------------------*/
#define USARTx  USART2


/*----------------------------------------------------------------------------
 Define  Baudrate setting (BRR) for USART
 *----------------------------------------------------------------------------*/
#define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))


/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
void SER_Initialize (void) {

#ifdef __DBG_ITM
  ITM_RxBuffer = ITM_RXBUFFER_EMPTY;       /*  CMSIS Debug Input              */
#else
  RCC->APB2ENR |=  (   1ul <<  2);         /* Enable GPIOA clock              */
  RCC->APB1ENR |=  (   1ul << 17);         /* Enable USART#2 clock            */

  /* Configure PA3 to USART2_RX, PA2 to USART2_TX */
  RCC->APB2ENR |=  (   1ul <<  0);         /* enable clock Alternate Function */
  AFIO->MAPR   &= ~(   1ul <<  3);         /* clear USART2 remap              */

  GPIOA->CRL   &= ~(0xFFul <<  8);         /* clear PA2, PA3                  */
  GPIOA->CRL   |=  (0x0Bul <<  8);         /* USART2 Tx (PA2) output push-pull*/
  GPIOA->CRL   |=  (0x04ul << 12);         /* USART2 Rx (PA3) input floating  */

  USARTx->BRR  = __USART_BRR(24000000ul, 115200ul);  /* 115200 baud @ 24MHz   */
  USARTx->CR3   = 0x0000;                  /* no flow control                 */
  USARTx->CR2   = 0x0000;                  /* 1 stop bit                      */
  USARTx->CR1   = ((   1ul <<  2) |        /* enable RX                       */
                   (   1ul <<  3) |        /* enable TX                       */
                   (   0ul << 12) |        /* 1 start bit, 8 data bits        */
                   (   1ul << 13) );       /* enable USART                    */
									 
//	 RCC->APB2ENR |=  (   1ul <<  2);         /* Enable GPIOA clock              */
  RCC->APB2ENR |=  (   1ul << 14);         /* Enable USART#2 clock            */

  /* Configure PA3 to USART2_RX, PA2 to USART2_TX */
  //RCC->APB2ENR |=  (   1ul <<  0);         /* enable clock Alternate Function */
  AFIO->MAPR   &= ~(   1ul <<  2);         /* clear USART2 remap              */

  GPIOA->CRH   &= ~(0xFFul <<  4);         /* clear PA2, PA3                  */
  GPIOA->CRH   |=  (0x0Bul <<  4);         /* USART2 Tx (PA2) output push-pull*/
  GPIOA->CRH   |=  (0x04ul << 8);         /* USART2 Rx (PA3) input floating  */

//  GPIOB->CRL   &= ~(0xFFul <<  24);         /* clear PA2, PA3                  */
//  GPIOB->CRL   |=  (0x0Bul <<  24);         /* USART2 Tx (PA2) output push-pull*/
//  GPIOB->CRL   |=  (0x04ul << 28);         /* USART2 Rx (PA3) input floating  */

  USART1->BRR  = __USART_BRR(24000000ul, 57600ul);  /* 115200 baud @ 24MHz   */
  USART1->CR3   = 0x0000;                  /* no flow control                 */
  USART1->CR2   = 0x0000;                  /* 1 stop bit                      */
  USART1->CR1   = ((   1ul <<  2) |        /* enable RX                       */
                   (   1ul <<  3) |        /* enable TX                       */
                   (   0ul << 12) |        /* 1 start bit, 8 data bits        */
                   (   1ul << 13) );       /* enable USART                    */
									 
		 RCC->APB2ENR |=  (   1ul <<  3);         /* Enable GPIOA clock              */
  RCC->APB1ENR |=  (   1ul << 18);         /* Enable USART#2 clock            */

  /* Configure PA3 to USART2_RX, PA2 to USART2_TX */
  //RCC->APB2ENR |=  (   1ul <<  0);         /* enable clock Alternate Function */
  AFIO->MAPR   &= ~(   3ul <<  4);         /* clear USART2 remap              */

  GPIOB->CRH   &= ~(0xFFul <<  8);         /* clear PA2, PA3                  */
  GPIOB->CRH   |=  (0x0Bul <<  8);         /* USART2 Tx (PA2) output push-pull*/
  GPIOB->CRH   |=  (0x04ul << 12);         /* USART2 Rx (PA3) input floating  */

  USART3->BRR  = __USART_BRR(24000000ul, 115200ul);  /* 115200 baud @ 24MHz   */
  USART3->CR3   = 0x0000;                  /* no flow control                 */
  USART3->CR2   = 0x0000;                  /* 1 stop bit                      */
  USART3->CR1   = ((   1ul <<  2) |        /* enable RX                       */
                   (   1ul <<  3) |        /* enable TX                       */
                   (   0ul << 12) |        /* 1 start bit, 8 data bits        */
                   (   1ul << 13) );       /* enable USART                    */
#endif
}


/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int SER_PutChar (int ch) {

#ifdef __DBG_ITM
  ITM_SendChar (ch & 0xFF);
#else
  while (!(USARTx->SR & 0x0080));
  USARTx->DR = (ch & 0xFF);
#endif
	
	 while (!(USART1->SR & 0x0080));
  USART1->DR = (ch & 0xFF);

  return (ch);
}

/*----------------------------------------------------------------------------
  Read character from Serial Port
 *----------------------------------------------------------------------------*/
int SER_GetChar (void) {

#ifdef __DBG_ITM
  if (ITM_CheckChar())
    return ITM_ReceiveChar();
#else
  if (USARTx->SR & 0x0020)
    return (USARTx->DR);
#endif

  return (-1);
}

