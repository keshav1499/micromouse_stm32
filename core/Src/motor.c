/*
 * motor.c
 *
 *  Created on: Jan 18, 2024
 *      Author: Keshav
 */
#include "main.h"
#include "motor.h"
#include "encoders.h"
#include "stm32f3xx_hal_tim.h"
#include "mpu6050.h"
//#include <stdio.h>

void forward(void){
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);//Right backward
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);//Left Backward

   //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);//Left Forward
   //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);//Right Forward

   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,4095);//Right Forward
   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4095);//Left Forward

}

void right(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);

	   //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);

	   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);//Right Forward
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4095);//Left Forward

}

void left(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);

	   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,4095);//Right Forward
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//Left Forward


}

void follow_left_wall(uint16_t IR1_RAW){
	forward();
	   if(IR1_RAW<2500){

		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 2000);//Right Forward //1.2*IR1_RAW

         goto JUMP;
	   }

    if(IR1_RAW>2600){

    	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 3700);//Left Forward//1.2*(4095-IR1_RAW)

    	   }
    JUMP:;
}

void follow_right_wall(uint8_t IR2_RAW){
	forward();
	   if(IR2_RAW){

		   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 3900);//Right Forward //1.2*IR1_RAW

         goto JUMP;
	   }

	   else {

    	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 3750);//Left Forward//1.2*(4095-IR1_RAW)

    	   }
    JUMP:;
}




void reverse(void){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);//Right backward
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);//Left Backward

	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);//Left Forward
	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);//Right Forward
}



void EncodedLine(int encR, int encL){

	forward();
	if(encR>encL){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);//Right Forward

	}

	forward();

	if(encL>encR){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);//Left Forward

		}

    forward();



}


void stop(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);//Right backward
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);//Left Backward

	   //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);//Left Forward
	   //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);//Right Forward

	   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);//Right Forward
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//Left Forward

}


void subtle_right90(){stop();
    Init_MPU6050();
    Get_Gyro_Cal_Values();
	right();

	float temp=Get_Yaw_Angle();
	while(Get_Yaw_Angle()-temp>=-80){
		//printf("%f\r\n",Angle_Yaw_Gyro);
	}
	  stop();

}







