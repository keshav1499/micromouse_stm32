/*
 * motor.h
 *
 *  Created on: Jan 18, 2024
 *      Author: Jon
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
extern TIM_HandleTypeDef htim3;extern TIM_HandleTypeDef htim1;


void forward(void);
void right(void);
void left(void);
void follow_left_wall(uint16_t IR1_RAW);
void follow_right_wall(uint8_t IR2_RAW);
void EncodedLine(int encR, int encL);

void subtle_right90();
void stop(void);

#endif /* INC_MOTOR_H_ */
