/*
 * encoders.c
 *
 *  Created on: Mar 2, 2024
 *      Author: Keshav
 */

#include "main.h"
#include "encoders.h"

 // Motor Right Forward pin PB0
 // Motor Right Reverse pin PA4

 // Motor Left Forward pin PC0
 // Motor Left Reverse pin PC1

int encoderPin1 = 2; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncodedR = 0,lastEncodedL = 0; // Here updated value of encoder store.
volatile long encoderValueR = 0,encoderValueL = 0; // Raw encoder value




  //pinMode(MotFwd, OUTPUT);
  //pinMode(MotRev, OUTPUT);
  //Serial.begin(9600); //initialize serial comunication

   //pinMode(encoderPin1, INPUT_PULLUP);
  //pinMode(encoderPin2, INPUT_PULLUP);

  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);//digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);//digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  //HAL_GPIO_EXTI_Callback(GPIO_P);//attachInterrupt(0, updateEncoder, CHANGE);
  //attachInterrupt(1, updateEncoder, CHANGE);




/*void loop() {

for (int i = 0; i <= 500; i++){
digitalWrite(MotFwd, LOW);
 digitalWrite(MotRev, HIGH);
 Serial.print("Forward  ");
 Serial.println(encoderValue);
}

delay(1000);

for (int i = 0; i <= 500; i++){
digitalWrite(MotFwd, HIGH);
 digitalWrite(MotRev, LOW);
 Serial.print("Reverse  ");
 Serial.println(encoderValue);
}

delay(1000);

}
*/
int encoderR(){return encoderValueR;}

int encoderL(){return encoderValueL;}

void updateEncoderRight(){
  int MSBR = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1); //MSB = most significant bit
  int LSBR = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15); //LSB = least significant bit

  int encodedR = (MSBR << 1) |LSBR; //converting the 2 pin value to single number
  int sumR  = (lastEncodedR << 2) | encodedR; //adding it to the previous encoded value

  if(sumR == 0b1101 || sumR == 0b0100 || sumR == 0b0010 || sumR == 0b1011) encoderValueR --;
  if(sumR == 0b1110 || sumR == 0b0111 || sumR == 0b0001 || sumR == 0b1000) encoderValueR ++;

  lastEncodedR = encodedR; //store this value for next time

}

void updateEncoderLeft(){
  int MSBL = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14); //MSB = most significant bit
  int LSBL = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13); //LSB = least significant bit

  int encodedL = (MSBL << 1) |LSBL; //converting the 2 pin value to single number
  int sumL  = (lastEncodedL << 2) | encodedL; //adding it to the previous encoded value

  if(sumL == 0b1101 || sumL == 0b0100 || sumL == 0b0010 || sumL == 0b1011) encoderValueL --;
  if(sumL == 0b1110 || sumL == 0b0111 || sumL == 0b0001 || sumL == 0b1000) encoderValueL ++;

  lastEncodedL = encodedL; //store this value for next time
}
