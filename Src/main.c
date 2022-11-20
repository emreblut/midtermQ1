/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "MyHeader.h"

int main(void)
{

	ENABLE_CLOCK_AHB1(RCC_AHB1ENR_GPIOEEN);	// Enable GPIOE Clock

	GPIOE->MODER |= ( GPIO_MODER_OUTPUT << ( 2 * GPIO_BSRR_BS2_Pos ) );
//	GPIOE->BSRR |= GPIO_BSRR_BS2;
    /* Loop forever */
	for(;;);
}