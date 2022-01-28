/*
 * stm32401xe_tim.c
 *
 *  Created on: Jan 21, 2022
 *      Author: ROJEK
 */
#include <stm32f401xe_tim.h>
#include "stm32f401xe_rcc.h"


void TIM_ClockEnable(TIM_TypeDef *p_timx)
{
	if (p_timx == TIM1)
	{
		RCC_CLOCK_TIM1_ENABLE();
	}
	else if (p_timx == TIM2)
	{
		RCC_CLOCK_TIM2_ENABLE();
	}
	else if (p_timx == TIM3)
	{
		RCC_CLOCK_TIM3_ENABLE();
	}
	else if (p_timx == TIM4)
	{
		RCC_CLOCK_TIM4_ENABLE();
	}
	else if (p_timx == TIM5)
	{
		RCC_CLOCK_TIM5_ENABLE();
	}
	else if (p_timx == TIM9)
	{
		RCC_CLOCK_TIM9_ENABLE();
	}else if (p_timx == TIM10)
	{
		RCC_CLOCK_TIM10_ENABLE();
	}else if (p_timx == TIM11)
	{
		RCC_CLOCK_TIM11_ENABLE();
	}

	//this operation is unnecessary here because configuration library is taking more than 2 clock cycles
	//between clock enable and configuring register, i leave it here to remind myself that stmf401x has
	//a limitation that is described in errata point 2.1.6
	__DSB();

}

void TIM_InitTimer(TimerHandle_t *p_handle_timer)
{
	TIM_ClockEnable(p_handle_timer->p_timx);
	// reset flags
	p_handle_timer->p_timx->SR = 0;

	// set prescaler
	p_handle_timer->p_timx->PSC = p_handle_timer->timer_config.prescaler;

	// set arr
	p_handle_timer->p_timx->ARR = p_handle_timer->timer_config.autoreload;

	// start timer
	p_handle_timer->p_timx->CR1 |= TIM_CR1_CEN;
}

void TIM_EnableIRQ(TimerHandle_t *p_handle_timer, uint8_t irq_flag)
{
	p_handle_timer->p_timx->DIER |= (0x01 << irq_flag);
}

void TIM_ClearUpdateFlag(TIM_TypeDef *p_timx)
{
	p_timx->SR &= ~(TIM_SR_UIF);
}

void TIM_StartTimer(TimerHandle_t *p_handle_timer)
{
	p_handle_timer->p_timx->CR1 |= TIM_CR1_CEN;
}
