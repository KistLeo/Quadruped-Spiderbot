/*
 * Servo.h
 *
 *  Created on: Mar 25, 2024
 *      Author: caoph
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "pca9685.h"
//#include "stm32f4xx_hal.h"
#include "stdio.h"

#define MIN_PWM 102
#define MAX_PWM 520

typedef struct  {
	uint8_t ID;
	uint16_t Min_Angle;
	uint16_t Max_Angle;
	uint8_t isForward;
	float Current_Angle;
	float Target_Angle;
	float Angle_offset;
}Servo_t;

extern Servo_t Servo0;
extern Servo_t Servo1;
extern Servo_t Servo2;
extern Servo_t Servo3;
extern Servo_t Servo4;
extern Servo_t Servo5;
extern Servo_t Servo6;
extern Servo_t Servo7;
extern Servo_t Servo8;
extern Servo_t Servo9;
extern Servo_t Servo10;
extern Servo_t Servo11;

PCA9685_STATUS Set_Servo_Angle(uint8_t Servo_ID, uint8_t isForward, float Angle);
PCA9685_STATUS ControlServoDirect(Servo_t *Servo, float Angle);
void ControlServo(Servo_t *Servo, float Target_Angle);

//typedef struct Servo Servo_t;


#endif /* INC_SERVO_H_ */
