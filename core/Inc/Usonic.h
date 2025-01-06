/*
 * Usonic.h
 *
 *  Created on: Mar 14, 2024
 *      Author: Jon
 */

#ifndef INC_USONIC_H_
#define INC_USONIC_H_

void K_Delay(uint16_t time);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HCSR04_Read (void);
uint8_t SonicDistance(void);

#endif /* INC_USONIC_H_ */
