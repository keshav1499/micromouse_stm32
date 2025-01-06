/*
 * MPU6050.h
 *
 *  Created on: May 26, 2023
 *      Author: Keshav
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include "stm32f3xx_hal.h"
#include <math.h>
#include "stdio.h"
#define MPU6050_ADRR 0X68<<1
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCE_CONFIG 0x1C
#define MPU6050_ACCE_MESURE 0x3B
#define MPU6050_GYRO_MESURE 0x43
#define MPU6050_POWER_1 0x6B
#define MPU6050_CONFIG 0x1A

extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart2;



uint8_t Init_MPU6050(void);
void Get_Gyro_Cal_Values(void);
void Get_All_Values(void);
float Get_Pitch_Angle(void);
float Get_Roll_Angle(void);
float Get_Yaw_Angle(void);
//void UART_Data_Transmitter(float Transmit_Data);
#endif /* INC_MPU6050_H_ */
