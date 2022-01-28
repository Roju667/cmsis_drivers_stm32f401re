/*
 * stm32401xe_tim.h
 *
 *  Created on: Jan 21, 2022
 *      Author: ROJEK
 */
#include "stm32f401xe.h"

typedef struct TimerConfig_t
{

	uint16_t prescaler;

	uint16_t autoreload;


}TimerConfig_t;


typedef struct TimerHandle_t
{
	TimerConfig_t timer_config;

	TIM_TypeDef *p_timx;
}TimerHandle_t;

#define TIM_IRQFLAG_UIE			0U
#define TIM_IRQFLAG_CC1IE		1U
#define TIM_IRQFLAG_CC2IE		2U
#define TIM_IRQFLAG_TIE			6U

void TIM_InitTimer(TimerHandle_t *p_handle_timer);
void TIM_StartTimer(TimerHandle_t *p_handle_timer);
void TIM_EnableIRQ(TimerHandle_t *p_handle_timer, uint8_t irq_flag);
void TIM_ClearUpdateFlag(TIM_TypeDef *p_timx);
