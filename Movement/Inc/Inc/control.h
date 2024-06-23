/*
 * control.h
 *
 *  Created on: Mar 20, 2024
 *      Author: caoph
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "pca9685.h"
#include "servo.h"
#include "math.h"


//TK xinh đẹp
//Servo0: chân trước bên trái -> Leg 0
//Leg 0: Trái trước
#define SERVO0 0
#define SERVO1 1
#define SERVO2 2
//Leg 1: Trái sau
#define SERVO3 4
#define SERVO4 5
#define SERVO5 6
//Leg 2: Phải sau
#define SERVO6 8
#define SERVO7 9
#define SERVO8 10
//Leg 3: Phải trước
#define SERVO9 13
#define SERVO10 14
#define SERVO11 15


typedef struct  {
	uint8_t ID;
	float x, y, z;
	float target_x, target_y, target_z;
	float theta1, theta2, theta3;
	float theta1_Servo, theta2_Servo, theta3_Servo;
	float theta1_offset;
	float theta2_offset;
	float theta3_offset;
}Leg_t;

extern Leg_t LegLF;
extern Leg_t LegLR;
extern Leg_t LegRF;
extern Leg_t LegRR;

void Spider_Init();
void Spider_Sit();
void Spider_stand();
void Spider_Step_Forward();
void Spider_Step_Forward_Arg(float x, float y, float z);
void Spider_Step_Backward_Arg(float x, float y, float z);
void Spider_Step_Forward_Yaw_Arg(float x, float y, float z);
void InverseKinematics(Leg_t *Leg, float x, float y, float z);
void InverseDirect(Leg_t *Leg, float x, float y, float z);
void Move_Body();
void Spider_Turn_Left();
void Spider_Turn_Right();
void PID(float Roll_value, float Pitch_value, float Time);
float PID_Yaw(float Yaw_value, float Time);
void ControlLeg(Leg_t *Leg);
void inverse(float x, float y, float z);
void Inverse_Calc(Leg_t *Leg, float x, float y, float z);

//PCA9685_STATUS controlServo0(float angle);
//PCA9685_STATUS controlServo1(float angle);
//PCA9685_STATUS controlServo2(float angle);
//PCA9685_STATUS controlServo3(float angle);
//PCA9685_STATUS controlServo4(float angle);
//PCA9685_STATUS controlServo5(float angle);
//PCA9685_STATUS controlServo6(float angle);
//PCA9685_STATUS controlServo7(float angle);
//PCA9685_STATUS controlServo8(float angle);
//PCA9685_STATUS controlServo9(float angle);
//PCA9685_STATUS controlServo10(float angle);
//PCA9685_STATUS controlServo11(float angle);


#endif /* INC_CONTROL_H_ */
