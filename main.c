/*
Copyright (C) 2018  Andrew Bernard

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

/*
 * UART,EXTI0 INTERRUPT, UART INTERRUPT TRANSFER COMPLETE --> TEST PROGRAM FOR STM32F103C8xx
 * for UART instance please refer to STM32F103C8xx datasheet UART/USART SECTION
 *
 * if you want porting this code to another series of STM32 please refer to that MCU datasheet
 * to make modification
 */

/* README PLEASE!!! 
The program continuously send random result for cos() function.
 * if EXTI0 interrupt happen the UART module will send "INTERRUPT!!!" word
 * the EXTI0 interrupt happen on falling edge in PIN A0 or PA0
 * The built GREEN Light in blue pill module or LED in PC13 will blink
 * if the function HAL_UART_Transmit_IT() complete to send T character
 */

#include "stm32f1xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* global variable */
UART_HandleTypeDef uart_conf;
GPIO_InitTypeDef gpio_conf;

/* DEBUG PURPOSE
uint8_t uart_data;
uint8_t ln_char='\n';
uint8_t cr_char='\r';
*/

int osc_clock_sys_config(void);
void uart_string(uint8_t *data);
unsigned char data[512];

int main(void)
{
	int delay;
	srand(512); /* random seed */
	if(osc_clock_sys_config() != 1)
	{
		return -1;
	}
	/* HAL INITIALIZATION */
	HAL_Init();
	HAL_SYSTICK_Config(HAL_RCC_GetSysClockFreq()/1000); /* FOR HAL TIMEBASE */

	/* ENABLE CLOCK FOR PERIPHERAL */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* UART GPIO INIT PA9 --> Tx | PA10 --> Rx */
	gpio_conf.Pin = GPIO_PIN_9;
	gpio_conf.Mode = GPIO_MODE_AF_PP;
	gpio_conf.Pull = GPIO_PULLUP;
	gpio_conf.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA,&gpio_conf);

	gpio_conf.Pin = GPIO_PIN_10;
	gpio_conf.Mode = GPIO_MODE_AF_INPUT;
	gpio_conf.Pull = GPIO_NOPULL;
	gpio_conf.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA,&gpio_conf);

	/* EXTI0 INTERRUPT */
	/* PA0 --> falling edge */
	gpio_conf.Pin = GPIO_PIN_0;
	gpio_conf.Mode = GPIO_MODE_IT_FALLING;
	gpio_conf.Pull = GPIO_PULLUP;
	gpio_conf.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA,&gpio_conf);

	/* PC13 LED TOGGLE */
	gpio_conf.Pin = GPIO_PIN_13;
	gpio_conf.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_conf.Pull = GPIO_PULLUP;
	gpio_conf.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC,&gpio_conf);

	/* INTERRUPT CONTROL */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI0_IRQn,0,0);
	HAL_NVIC_SetPriority(USART1_IRQn,0,1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	/* UART1 INIT */
	uart_conf.Init.BaudRate = 9600;
	uart_conf.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart_conf.Init.Mode = UART_MODE_TX_RX;
	uart_conf.Init.OverSampling = UART_OVERSAMPLING_16;
	uart_conf.Init.Parity = UART_PARITY_NONE;
	uart_conf.Init.StopBits = UART_STOPBITS_1;
	uart_conf.Init.WordLength = UART_WORDLENGTH_8B;
	uart_conf.Instance = USART1;
	HAL_UART_Init(&uart_conf);

	sprintf((char * restrict)data,"HELLO WORLD!!! --> %f \r\n",3.323);
	uart_string(data);

	while(1)
	{
		sprintf((char * restrict)data,"random cos() --> %3.4f \r\n",cos(rand()));
		uart_string(data);
		HAL_UART_Transmit_IT(&uart_conf,"T",sizeof(uint8_t));
		for(delay=0;delay <= 10000;delay++){} /* DELAY ROUTINES */
	}
}

int osc_clock_sys_config(void)
{
/* CPU SPEED CONFIG : 32MHz (HSE--> 8MHz PLL=HSE*4) */
RCC_OscInitTypeDef osc_conf;
RCC_ClkInitTypeDef clk_conf;

/* Oscillator Initialization */
osc_conf.OscillatorType = RCC_OSCILLATORTYPE_HSE;
osc_conf.HSEState = RCC_HSE_ON;
osc_conf.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
osc_conf.PLL.PLLState = RCC_PLL_ON ;
osc_conf.PLL.PLLSource = RCC_PLLSOURCE_HSE;
osc_conf.PLL.PLLMUL = RCC_PLL_MUL4;
	if(HAL_RCC_OscConfig(&osc_conf) != HAL_OK)
	{
		return 0;
	}

/* System Clock Config */
clk_conf.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
clk_conf.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
clk_conf.AHBCLKDivider = RCC_SYSCLK_DIV1;
clk_conf.APB1CLKDivider = RCC_HCLK_DIV1;
clk_conf.APB2CLKDivider = RCC_HCLK_DIV1;
	if(HAL_RCC_ClockConfig(&clk_conf,FLASH_LATENCY_2) != HAL_OK)
	{
		return 0;
	}
return 1;
}

void uart_string(uint8_t *data)
{
	uint8_t tmp;
	unsigned int cnt;
	for(cnt=0;data[cnt] != '\0';cnt++)
	{
		tmp = data[cnt];
		HAL_UART_Transmit(&uart_conf,&tmp,sizeof(uint8_t),HAL_TIMEOUT);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) /* EXTI0 INTERRUPT ROUTINES */
{
	sprintf((char * restrict)data,"INTERRUPT!!!\r\n");
	uart_string(data);
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart) /* UART INTERRUPT ROUTINES */
{
	int cnt,cnt_1;
		for(cnt_1 = 0;cnt_1 <= 20;cnt_1++)
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			for(cnt=0;cnt <= 10000;cnt++){} /* DELAY ROUTINES */
		}
}
