/*
 * stmf401xe_systick.h
 *
 *  Created on: 14 lut 2022
 *      Author: ROJEK
 */


void SYSTICK_ConfigureMilisecond(void);
void SYSTICK_Delay(uint32_t miliseconds);
uint32_t SYSTICK_GetTick(void);
