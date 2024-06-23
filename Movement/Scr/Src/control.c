/*
 * control.c
 *
 *  Created on: Mar 20, 2024
 *      Author: caoph
 */

#include "control.h"
//#include "pca9685.h"
//#include "math.h"
//#include "servo.h"


//Leg length
//a1 = 56.84
//a2 = 114
//a3 = 170.74

#define a1 56.84
#define a2 114.0
#define a3 170.74
#define PI 3.1415926
#define Standsteps 40
#define Z_Stand -40
#define InitSteps 40
#define SitSteps 40
#define MoveSteps 30
#define Movebody 12
#define T 100
#define T_control_leg 4
#define z_up -20
#define x_init 160
#define z_init -80
#define x_offset 20

#define Mode_Walk_Foward
#define Mode_Walk_Backward
#define Mode_Turn_Left
#define Mode_Turn_Right
#define Mode_Balance

Servo_t *array_Servo[12] = {&Servo0, &Servo1, &Servo2, &Servo3, &Servo4, &Servo5, &Servo6, &Servo7, &Servo8,
				&Servo9, &Servo10, &Servo11};
Leg_t *array_Leg[4] = {&LegLF, &LegLR, &LegRR, &LegRF};

float startAngle[12] = { 90, 175, 5, 95, 175, 5, 140, 175, 5, 48, 175, 5 };
float currentAngle[12] = { 90, 120, 10, 95, 110, 10, 140, 120, 10, 48, 120, 10 };
float standAngle[12] = { 90, 120, 10, 95, 110, 10, 140, 120, 10, 48, 120, 10 };

float PosLF[3], ThetaLF[3]; //left front, [x, y, z]
float PosLR[3], ThetaLR[3];	//lefr rear
float PosRF[3], ThetaRF[3];
float PosRR[3], ThetaRR[3];
//Control on GUI
extern uint8_t Mode;
//PID
float Roll_E0, Roll_E1, Roll_E2;
float Pitch_E0, Pitch_E1, Pitch_E2;
float Yaw_E0, Yaw_E1, Yaw_E2;
float Roll_Ref = 0, Pitch_Ref = 5, Yaw_Ref;
float Roll_Pre, Pitch_Pre, Yaw_Pre;
float Roll_u, Roll_u1, Pitch_u, Pitch_u1, Yaw_u, Yaw_u1;
float Kp = 0.1;
float Ki;
float Kd;
float Leg1_u1, Leg2_u1, Leg3_u1, Leg4_u1;
float Leg1_u, Leg2_u, Leg3_u, Leg4_u;

Leg_t LegLF = {
		.ID = 1,
		.x = 138, .target_x = 125,
		.y = 0, .target_y = 75,
		.z = 0, .target_z = -27,
		.theta1_offset = 45,
		.theta2_offset = 70,
		.theta3_offset = 138
};
Leg_t LegLR = {
		.ID = 2,
		.x = 138, .target_x = 125,
		.y = 0, .target_y = -75,
		.z = 0, .target_z = -27,
		.theta1_offset = 141,
		.theta2_offset = 76,
		.theta3_offset = 132
};
Leg_t LegRR = {
		.ID = 3,
		.x = 138, .target_x = 125,
		.y = 0, .target_y = 0,
		.z = 0, .target_z = -27,
		.theta1_offset = 134,
		.theta2_offset = 77,
		.theta3_offset = 132
};
Leg_t LegRF = {
		.ID = 4,
		.x = 138, .target_x = 125,
		.y = 0, .target_y = 0,
		.z = 0, .target_z = -27,
		.theta1_offset = 43,
		.theta2_offset = 74,
		.theta3_offset = 138
};
void Spider_Init(){
	InverseDirect(&LegLF, LegLF.target_x, LegLF.target_y, LegLF.target_z);
	InverseDirect(&LegLR, LegLR.target_x, LegLR.target_y, LegLR.target_z);
	InverseDirect(&LegRF, LegRF.target_x, LegRF.target_y, LegRF.target_z);
	InverseDirect(&LegRR, LegRR.target_x, LegRR.target_y, LegRR.target_z);
	osDelay(2000);
	//HAL_Delay(20);

	//float target_x = 137, target_y = 0, target_z = -120;
	float target_x = 125, target_y = 0, target_z = -55;
	float step[3];
	step[0] = (target_x - LegLF.x)/InitSteps;
	step[2] = (target_z - LegLF.z)/InitSteps;
	for(float i = 0; i < InitSteps; i++){
		InverseDirect(&LegLF, LegLF.x + step[0], LegLF.y, LegLF.z + step[2]);
		InverseDirect(&LegLR, LegLR.x + step[0], LegLR.y, LegLR.z + step[2]);
		InverseDirect(&LegRF, LegRF.x + step[0], LegRF.y, LegRF.z + step[2]);
		InverseDirect(&LegRR, LegRR.x + step[0], LegRR.y, LegRR.z + step[2]);
		//HAL_Delay(25);
		osDelay(25);
	}
	for(int i=0; i<4; i++){
		array_Leg[i]->target_x = array_Leg[i]->x;
		array_Leg[i]->target_y = array_Leg[i]->y;
		array_Leg[i]->target_z = array_Leg[i]->z;
	}

}
void Spider_Sit(){
//	InverseDirect(&LegLF, LegLF.target_x, LegLF.target_y, LegLF.target_z);
//	InverseDirect(&LegLR, LegLR.target_x, LegLR.target_y, LegLR.target_z);
//	InverseDirect(&LegRF, LegRF.target_x, LegRF.target_y, LegRF.target_z);
//	InverseDirect(&LegRR, LegRR.target_x, LegRR.target_y, LegRR.target_z);
//	HAL_Delay(2000);
//	float target_x = 125, target_y = 0, target_z = -50;
//	float step[3];
//	step[0] = (target_x - LegLF.x)/SitSteps;
//	step[2] = (target_z - LegLF.z)/SitSteps;
//	for(float i = 0; i < SitSteps; i++){
//		InverseDirect(&LegLF, LegLF.x + step[0], target_y + 75, LegLF.z + step[2]);
//		InverseDirect(&LegLR, LegLR.x + step[0], target_y - 75, LegLR.z + step[2]);
//		InverseDirect(&LegRF, LegRF.x + step[0], target_y, LegRF.z + step[2]);
//		InverseDirect(&LegRR, LegRR.x + step[0], target_y, LegRR.z + step[2]);
//		osDelay(25);
//	}
//	for(int i=0; i<4; i++){
//		array_Leg[i]->target_x = array_Leg[i]->x;
//		array_Leg[i]->target_y = array_Leg[i]->y;
//		array_Leg[i]->target_z = array_Leg[i]->z;
//	}
	float target_x = 125, target_y = 0, target_z = -55;
	Inverse_Calc(&LegLF, target_x, target_y + 75, target_z);
	Inverse_Calc(&LegLR, target_x, target_y - 75, target_z);
	Inverse_Calc(&LegRR, target_x, target_y, target_z);
	Inverse_Calc(&LegRF, target_x, target_y, target_z);
	for(int leg = 0; leg < 4; leg++){
		array_Servo[leg*3 + 0]->Target_Angle = array_Leg[leg]->theta1_Servo;
		array_Servo[leg*3 + 1]->Target_Angle = array_Leg[leg]->theta2_Servo;
		array_Servo[leg*3 + 2]->Target_Angle = array_Leg[leg]->theta3_Servo;
	}
	float step[12];
	for(int i=0; i<12; i++){
		step[i] = (array_Servo[i]->Target_Angle - array_Servo[i]->Current_Angle)/SitSteps;
	}
	for(int i=0; i < SitSteps; i++){
		for(int servo_id = 0; servo_id < 12; servo_id++){
			ControlServoDirect(array_Servo[servo_id], array_Servo[servo_id]->Current_Angle + step[servo_id]);
		}
		osDelay(30);
	}
	for(int i=0; i<4; i++){
		array_Leg[i]->x = array_Leg[i]->target_x;
		array_Leg[i]->y = array_Leg[i]->target_y;
		array_Leg[i]->z = array_Leg[i]->target_z;
	}
}
void Spider_stand() {
	float target_x = 160, target_y = 0, target_z = -80;
	Inverse_Calc(&LegLF, target_x, target_y + 76, target_z);
	Inverse_Calc(&LegLR, target_x, target_y - 76, target_z);
	Inverse_Calc(&LegRR, target_x, target_y, target_z);
	Inverse_Calc(&LegRF, target_x, target_y, target_z);
	for(int leg = 0; leg < 4; leg++){
		array_Servo[leg*3 + 0]->Target_Angle = array_Leg[leg]->theta1_Servo;
		array_Servo[leg*3 + 1]->Target_Angle = array_Leg[leg]->theta2_Servo;
		array_Servo[leg*3 + 2]->Target_Angle = array_Leg[leg]->theta3_Servo;
	}
	float step[12];
	for(int i=0; i<12; i++){
		step[i] = (array_Servo[i]->Target_Angle - array_Servo[i]->Current_Angle)/Standsteps;
	}
	for(int i=0; i < Standsteps; i++){
		for(int servo_id = 0; servo_id < 12; servo_id++){
			ControlServoDirect(array_Servo[servo_id], array_Servo[servo_id]->Current_Angle + step[servo_id]);
		}
		osDelay(30);
	}


//	step[0] = (target_x - LegLF.x)/Standsteps;
//	step[2] = (target_z - LegLF.z)/Standsteps;
//	for(float i = 0; i < Standsteps; i++){
//		InverseDirect(&LegLF, LegLF.x + step[0], target_y + 75, LegLF.z + step[2]);
//		InverseDirect(&LegLR, LegLR.x + step[0], target_y - 75, LegLR.z + step[2]);
//		InverseDirect(&LegRF, LegRF.x + step[0], target_y, LegRF.z + step[2]);
//		InverseDirect(&LegRR, LegRR.x + step[0], target_y, LegRR.z + step[2]);
//		osDelay(25);
//	}
	for(int i=0; i<4; i++){
		array_Leg[i]->x = array_Leg[i]->target_x;
		array_Leg[i]->y = array_Leg[i]->target_y;
		array_Leg[i]->z = array_Leg[i]->target_z;
	}
}
void Spider_Step_Forward(){
	//Step 1
	Inverse_Calc(&LegRF, 137, 0, -120);
	ControlLeg(&LegRF);
	osDelay(T);
	Inverse_Calc(&LegRF, 137, 0, -40);
	ControlLeg(&LegRF);
	osDelay(T);
	Inverse_Calc(&LegRF, 137, 150, -40);
	ControlLeg(&LegRF);
	osDelay(T);
	Inverse_Calc(&LegRF, 137, 150, -120);
	ControlLeg(&LegRF);
	osDelay(T);
	//Step 2
	Inverse_Calc(&LegLF, 137, 0, -120);
	Inverse_Calc(&LegLR, 137, -150, -120);
	Inverse_Calc(&LegRR, 137, -75, -120);
	Inverse_Calc(&LegRF, 137, 75, -120);
	//Move_Body();

	ControlLeg(&LegLF);
	ControlLeg(&LegLR);
	ControlLeg(&LegRR);
	ControlLeg(&LegRF);
	HAL_Delay(T);
	//Step 3
	Inverse_Calc(&LegLR, 137, -150, -40);
	ControlLeg(&LegLR);
	HAL_Delay(T);
	Inverse_Calc(&LegLR, 137, 0, -40);
	ControlLeg(&LegLR);
	HAL_Delay(T);
	Inverse_Calc(&LegLR, 137, 0, -120);
	ControlLeg(&LegLR);
	HAL_Delay(T);
	//Step 4
	Inverse_Calc(&LegLF, 137, 0, -40);
	ControlLeg(&LegLF);
	HAL_Delay(T);
	Inverse_Calc(&LegLF, 137, 150, -40);
	ControlLeg(&LegLF);
	HAL_Delay(T);
	Inverse_Calc(&LegLF, 137, 150, -120);
	ControlLeg(&LegLF);
	HAL_Delay(T);
	//Step 5
	Inverse_Calc(&LegLF, 137, 75, -120);
	Inverse_Calc(&LegLR, 137, -75, -120);
	Inverse_Calc(&LegRR, 137, -150, -120);
	Inverse_Calc(&LegRF, 137, 0, -120);
	//Move_Body();

	ControlLeg(&LegLF);
	ControlLeg(&LegLR);
	ControlLeg(&LegRR);
	ControlLeg(&LegRF);
	HAL_Delay(T);
	//Step 6
	Inverse_Calc(&LegRR, 137, -150, -40);
	ControlLeg(&LegRR);
	HAL_Delay(T);
	Inverse_Calc(&LegRR, 137, 0, -40);
	ControlLeg(&LegRR);
	HAL_Delay(T);
	Inverse_Calc(&LegRR, 137, 0, -120);
	ControlLeg(&LegRR);
	HAL_Delay(T);
}
void Spider_Step_Forward_Arg(float x, float y, float z){
	//Step 1
//	Inverse_Calc(&LegRF, x + x_offset, 0, z);
//	ControlLeg(&LegRF);
	//HAL_Delay(T);
	if(Mode == 0){
		return;
	}
	Inverse_Calc(&LegRF, x, 0, z_up);
	ControlLeg(&LegRF);
	//HAL_Delay(T);
	Inverse_Calc(&LegRF, x, 2*y, z_up);
	ControlLeg(&LegRF);
	//HAL_Delay(T);
	Inverse_Calc(&LegRF, x, 2*y, z);
	ControlLeg(&LegRF);
	osDelay(T);
	//Step 2
	Inverse_Calc(&LegLF, x, 0, z);
	Inverse_Calc(&LegLR, x, -2*y, z);
	Inverse_Calc(&LegRR, x, -y, z);
	Inverse_Calc(&LegRF, x, y, z);
	Move_Body();

//	ControlLeg(&LegLF);
//	ControlLeg(&LegLR);
//	ControlLeg(&LegRR);
//	ControlLeg(&LegRF);
	//HAL_Delay(T);
	//Step 3
	Inverse_Calc(&LegLR, x, -2*y, z_up);
	ControlLeg(&LegLR);
	//HAL_Delay(T);
	Inverse_Calc(&LegLR, x, 0, z_up);
	ControlLeg(&LegLR);
	//HAL_Delay(T);
	Inverse_Calc(&LegLR, x, 0, z);
	ControlLeg(&LegLR);
	osDelay(T);
	//Step 4
	Inverse_Calc(&LegLF, x, 0, z_up);
	ControlLeg(&LegLF);
	//HAL_Delay(T);
	Inverse_Calc(&LegLF, x, 2*y, z_up);
	ControlLeg(&LegLF);
	//HAL_Delay(T);
	Inverse_Calc(&LegLF, x, 2*y, z);
	ControlLeg(&LegLF);
	osDelay(T);
	//Step 5
	Inverse_Calc(&LegLF, x, y, z);
	Inverse_Calc(&LegLR, x, -y, z);
	Inverse_Calc(&LegRR, x, -2*y, z);
	Inverse_Calc(&LegRF, x, 0, z);
	Move_Body();

//	ControlLeg(&LegLF);
//	ControlLeg(&LegLR);
//	ControlLeg(&LegRR);
//	ControlLeg(&LegRF);
	//HAL_Delay(T);
	//Step 6
	Inverse_Calc(&LegRR, x, -2*y, z_up);
	ControlLeg(&LegRR);
	//HAL_Delay(T);
	Inverse_Calc(&LegRR, x, 0, z_up);
	ControlLeg(&LegRR);
	//HAL_Delay(T);
	Inverse_Calc(&LegRR, x, 0, z);
	ControlLeg(&LegRR);
	osDelay(T);
}

void Spider_Step_Backward_Arg(float x, float y, float z){
	//Step 1
//	Inverse_Calc(&LegRR, x, 0, z);
//	ControlLeg(&LegRR);
	//HAL_Delay(T);
	if(Mode == 0){
		return;
	}
	Inverse_Calc(&LegRR, x, 0, z_up);
	ControlLeg(&LegRR);
	//HAL_Delay(T);
	Inverse_Calc(&LegRR, x, -2*y, z_up);
	ControlLeg(&LegRR);
	//HAL_Delay(T);
	Inverse_Calc(&LegRR, x, -2*y, z);
	ControlLeg(&LegRR);
	osDelay(T);
	//Step 2
	Inverse_Calc(&LegLF, x, 2*y, z);
	Inverse_Calc(&LegLR, x, 0, z);
	Inverse_Calc(&LegRR, x, -y, z);
	Inverse_Calc(&LegRF, x, y, z);
	Move_Body();

//	ControlLeg(&LegLF);
//	ControlLeg(&LegLR);
//	ControlLeg(&LegRR);
//	ControlLeg(&LegRF);
	//HAL_Delay(T);
	//Step 3
	Inverse_Calc(&LegLF, x, 2*y, z_up);
	ControlLeg(&LegLF);
	//HAL_Delay(T);
	Inverse_Calc(&LegLF, x, 0, z_up);
	ControlLeg(&LegLF);
	//HAL_Delay(T);
	Inverse_Calc(&LegLF, x, 0, z);
	ControlLeg(&LegLF);
	osDelay(T);
	//Step 4
	Inverse_Calc(&LegLR, x, 0, z_up);
	ControlLeg(&LegLR);
	//HAL_Delay(T);
	Inverse_Calc(&LegLR, x, -2*y, z_up);
	ControlLeg(&LegLR);
	//HAL_Delay(T);
	Inverse_Calc(&LegLR, x, -2*y, z);
	ControlLeg(&LegLR);
	osDelay(T);
	//Step 5
	Inverse_Calc(&LegLF, x, y, z);
	Inverse_Calc(&LegLR, x, -y, z);
	Inverse_Calc(&LegRR, x, 0, z);
	Inverse_Calc(&LegRF, x, 2*y, z);
	Move_Body();

//	ControlLeg(&LegLF);
//	ControlLeg(&LegLR);
//	ControlLeg(&LegRR);
//	ControlLeg(&LegRF);
	//HAL_Delay(T);
	//Step 6
	Inverse_Calc(&LegRF, x, 2*y, z_up);
	ControlLeg(&LegRF);
	//HAL_Delay(T);
	Inverse_Calc(&LegRF, x, 0, z_up);
	ControlLeg(&LegRF);
	//HAL_Delay(T);
	Inverse_Calc(&LegRF, x, 0, z);
	ControlLeg(&LegRF);
	osDelay(T);
}
void Spider_Turn_Left(){
//Step 1
//	Inverse_Calc(&LegRF, x_init, 0, z_init);
//	ControlLeg(&LegRF);
//	osDelay(T);
	Inverse_Calc(&LegRF, x_init, 0, z_up);
	ControlLeg(&LegRF);
	osDelay(T);

//	float step[3];
//
//	for(int i=0; i<12; i++){
//		step[i] = (array_Servo[i]->Target_Angle - array_Servo[i]->Current_Angle)/MoveSteps;
//	}
//	for(int i=0; i < MoveSteps; i++){
//			for(int servo_id = 0; servo_id < 12; servo_id++){
//				ControlServoDirect(array_Servo[servo_id], array_Servo[servo_id]->Current_Angle + step[servo_id]);
//	//			if(servo_id % 3 == 2)
//					//HAL_Delay(25);
//			}
//			HAL_Delay(5);
//		}
//	//ControlServoDirect(&Servo0, Servo0.Current_Angle - 45/4);
//	ControlServoDirect(&Servo3, Servo3.Current_Angle + 45/4);
//	ControlServoDirect(&Servo6, Servo6.Current_Angle + 45/4);
//	ControlServoDirect(&Servo9, Servo9.Current_Angle - 45/4);

	Inverse_Calc(&LegLF, 115.73, 105.23, z_init);
	Inverse_Calc(&LegLR, 149, -46.83, z_init);
	Inverse_Calc(&LegRR, 134.37, -26.73, z_init);
	Inverse_Calc(&LegRF, 110.44, 110.44, z_up);
	Move_Body();

	Inverse_Calc(&LegRF, 110.44, 110.44, z_init);
	ControlLeg(&LegRF);
	osDelay(T);

	//Step 2
	Inverse_Calc(&LegLF, 115.73, 105.23, z_up);
	ControlLeg(&LegLF);
	osDelay(T);

//	Inverse_Calc(&LegLF, 137, 0, z_up);
//	Inverse_Calc(&LegLR, 137, 0, z_init);
//	Inverse_Calc(&LegRR, 137, -75, z_init);
//	Inverse_Calc(&LegRF, 137, 75, z_init);
	Inverse_Calc(&LegLF, x_init, 0, z_up);
	Inverse_Calc(&LegLR, x_init, 0, z_init);
	Inverse_Calc(&LegRR, x_init, -76, z_init);
	Inverse_Calc(&LegRF, x_init, 76, z_init);
	Move_Body();
	Inverse_Calc(&LegLF, x_init, 0, z_init);
	ControlLeg(&LegLF);
	osDelay(T);

	//Step 3
	Inverse_Calc(&LegLR, x_init, 0, z_up);
	ControlLeg(&LegLR);
	osDelay(T);

	Inverse_Calc(&LegLF, 134.37, 26.73, z_init);
	Inverse_Calc(&LegLR, 100, -135, z_up);
	Inverse_Calc(&LegRR, 119.73, -100.23, z_init);
	Inverse_Calc(&LegRF, 145, 40, z_init);
	Move_Body();

	Inverse_Calc(&LegLR, 100, -135, z_init);
	ControlLeg(&LegLR);
	osDelay(T);

	//Step 4
	Inverse_Calc(&LegRR, 119.73, -100.23, z_up);
	ControlLeg(&LegRR);
	osDelay(T);

	Inverse_Calc(&LegLF, x_init, 76, z_init);
	Inverse_Calc(&LegLR, x_init, -76, z_init);
	Inverse_Calc(&LegRR, x_init, 0, z_up);
	Inverse_Calc(&LegRF, x_init, 0, z_init);
	Move_Body();

//	Inverse_Calc(&LegRR, x_init, 0, z_init);
//	Inverse_Calc(&LegLR, x_init, -100, z_init);
//	ControlLeg(&LegLR);
//	ControlLeg(&LegRR);

	Inverse_Calc(&LegRR, x_init, 0, z_init);
	ControlLeg(&LegRR);
	osDelay(T);
}
void Spider_Turn_Right(){
//Step 1
	Inverse_Calc(&LegRR, x_init, 0, z_init);
	ControlLeg(&LegRR);
	osDelay(T);
	Inverse_Calc(&LegRR, x_init, 0, z_up);
	ControlLeg(&LegRR);
	osDelay(T);

	Inverse_Calc(&LegLF, 149, 46.83, z_init);
	Inverse_Calc(&LegLR, 115.73, -105.23, z_init);
	Inverse_Calc(&LegRR, 110.44, -110.44, z_up);
	Inverse_Calc(&LegRF, 134.37, 26.73, z_init);
	Move_Body();
	Inverse_Calc(&LegRR, 110.44, -110.44, z_init);
	ControlLeg(&LegRR);
	osDelay(T);

	//Step 2
	Inverse_Calc(&LegLR, 115.73, -105.23, z_up);
	ControlLeg(&LegLR);
	osDelay(T);

	Inverse_Calc(&LegLF, x_init, 0, z_init);
	Inverse_Calc(&LegLR, x_init, 0, z_up);
	Inverse_Calc(&LegRR, x_init, -76, z_init);
	Inverse_Calc(&LegRF, x_init, 76, z_init);
	Move_Body();
	Inverse_Calc(&LegLR, x_init, 0, z_init);
	ControlLeg(&LegLR);
	osDelay(T);

	//Step 3
	Inverse_Calc(&LegLF, x_init, 0, z_up);
	ControlLeg(&LegLF);
	osDelay(T);

	Inverse_Calc(&LegLF, 100, 135, z_up);
	Inverse_Calc(&LegLR, 134.37, -26.73, z_init);
	Inverse_Calc(&LegRR, 145, -40, z_init);
	Inverse_Calc(&LegRF, 119.73, 100.23, z_init);
	Move_Body();

	Inverse_Calc(&LegLF, 100, 135, z_init);
	ControlLeg(&LegLF);
	osDelay(T);

	//Step 4
	Inverse_Calc(&LegRF, 119.73, 100.23, z_up);
	ControlLeg(&LegRF);
	osDelay(T);

	Inverse_Calc(&LegLF, x_init, 76, z_init);
	Inverse_Calc(&LegLR, x_init, -76, z_init);
	Inverse_Calc(&LegRR, x_init, 0, z_init);
	Inverse_Calc(&LegRF, x_init, 0, z_up);
	Move_Body();

//	Inverse_Calc(&LegRR, 137, 0, z_init);
//	Inverse_Calc(&LegLR, 137, -100, z_init);
//	ControlLeg(&LegLR);
//	ControlLeg(&LegRR);

	Inverse_Calc(&LegRF, x_init, 0, z_init);
	ControlLeg(&LegRF);
	osDelay(T);

}
void Move_Body(){
	for(int leg = 0; leg < 4; leg++){
		array_Servo[leg*3 + 0]->Target_Angle = array_Leg[leg]->theta1_Servo;
		array_Servo[leg*3 + 1]->Target_Angle = array_Leg[leg]->theta2_Servo;
		array_Servo[leg*3 + 2]->Target_Angle = array_Leg[leg]->theta3_Servo;
	}
	float step[12];

	for(int i=0; i<12; i++){
		step[i] = (array_Servo[i]->Target_Angle - array_Servo[i]->Current_Angle)/Movebody;
	}

	for(int i=0; i < Movebody; i++){
		for(int servo_id = 0; servo_id < 12; servo_id++){
			ControlServoDirect(array_Servo[servo_id], array_Servo[servo_id]->Current_Angle + step[servo_id]);
//			if(servo_id % 3 == 2)
				//HAL_Delay(1);
			//for(uint16_t i = 0; i<100; i++);
		}
		osDelay(30);
	}
	for(int i=0; i<4; i++){
		array_Leg[i]->x = array_Leg[i]->target_x;
		array_Leg[i]->y = array_Leg[i]->target_y;
		array_Leg[i]->z = array_Leg[i]->target_z;
	}
}

void Spider_Step_Forward_Yaw_Arg(float x, float y, float z){
	//Step 1
//	Inverse_Calc(&LegRF, x, 0, z);
//	ControlLeg(&LegRF);
	//HAL_Delay(T);
	Inverse_Calc(&LegRF, x, 0, z_up);
	ControlLeg(&LegRF);
	//HAL_Delay(T);
	Inverse_Calc(&LegRF, x, 2*y - Yaw_u, z_up);
	ControlLeg(&LegRF);
	//HAL_Delay(T);
	Inverse_Calc(&LegRF, x, 2*y - Yaw_u, z);
	ControlLeg(&LegRF);
	osDelay(T);
	//Step 2
	Inverse_Calc(&LegLF, x, 0 - Yaw_u, z);
	Inverse_Calc(&LegLR, x, -2*y - Yaw_u, z);
	Inverse_Calc(&LegRR, x, -y + Yaw_u, z);
	Inverse_Calc(&LegRF, x, y - Yaw_u, z);
	Move_Body();

//	ControlLeg(&LegLF);
//	ControlLeg(&LegLR);
//	ControlLeg(&LegRR);
//	ControlLeg(&LegRF);
	//HAL_Delay(T);
	//Step 3
	Inverse_Calc(&LegLR, x, -2*y - Yaw_u, z_up);
	ControlLeg(&LegLR);
	//HAL_Delay(T);
	Inverse_Calc(&LegLR, x, 0 + Yaw_u, z_up);
	ControlLeg(&LegLR);
	//HAL_Delay(T);
	Inverse_Calc(&LegLR, x, 0 + Yaw_u, z);
	ControlLeg(&LegLR);
	osDelay(T);
	//Step 4
	Inverse_Calc(&LegLF, x, 0 - Yaw_u, z_up);
	ControlLeg(&LegLF);
	//HAL_Delay(T);
	Inverse_Calc(&LegLF, x, 2*y + Yaw_u, z_up);
	ControlLeg(&LegLF);
	//HAL_Delay(T);
	Inverse_Calc(&LegLF, x, 2*y + Yaw_u, z);
	ControlLeg(&LegLF);
	osDelay(T);
	//Step 5
	Inverse_Calc(&LegLF, x, y + Yaw_u, z);
	Inverse_Calc(&LegLR, x, -y - Yaw_u, z);
	Inverse_Calc(&LegRR, x, -2*y + Yaw_u, z);
	Inverse_Calc(&LegRF, x, 0 - Yaw_u, z);
	Move_Body();

//	ControlLeg(&LegLF);
//	ControlLeg(&LegLR);
//	ControlLeg(&LegRR);
//	ControlLeg(&LegRF);
	//HAL_Delay(T);
	//Step 6
	Inverse_Calc(&LegRR, x, -2*y + Yaw_u, z_up);
	ControlLeg(&LegRR);
	//HAL_Delay(T);
	Inverse_Calc(&LegRR, x, 0 - Yaw_u, z_up);
	ControlLeg(&LegRR);
	//HAL_Delay(T);
	Inverse_Calc(&LegRR, x, 0 - Yaw_u, z);
	ControlLeg(&LegRR);
	osDelay(T);
}

//Return value theta in radian
void inverse(float x, float y, float z){
	//Don't use
	float L;
	float theta1, theta2, theta3, J3, A, B;
	theta1 = atan(y/x);
	L = sqrtf(pow(x,2) + pow(z,2));
	J3 = acos((pow(a2,2) + pow(a3,2) - pow(L,2)) / (2*a2*a3));
	theta3 = PI - J3;
	B = acos((pow(L,2) + pow(a2,2) - pow(a3,2)) / (2*L*a2));
	A = atan(z/x);
	theta2 = B - A;
}

//Theta is degree
void Inverse_Calc(Leg_t *Leg, float x, float y, float z){
	if(Mode == 0){
		return;
	}
	float L, x_23, y_23;
	float theta1, theta2, theta3, J3, A, B;
	if(y == 0){
		theta1 = 0;
	}
	else if(x == 0){
		theta1 = PI/2.0;
	}
	else
		theta1 = atan(y/x);
	//y1 = a1*sin(theta1);
	y_23 = y - a1*sin(theta1);
	x_23 = x - a1*cos(theta1);
	//float x_2 = x - a1;
	L = sqrtf(pow(x_23,2) + pow(y_23,2) + pow(z,2));
	J3 = acos((pow(a2,2) + pow(a3,2) - pow(L,2)) / (2*a2*a3));
	theta3 = J3 - PI;
	B = acos((pow(L,2) + pow(a2,2) - pow(a3,2)) / (2*L*a2));
	A = atan(-z/x_23);
	theta2 = B - A;
	//Convert theta to degree
	Leg->theta1 = theta1 * 180/PI;
	Leg->theta2 = theta2 * 180/PI;
	Leg->theta3 = theta3 * 180/PI;
	//Convert to servo angle
	Leg->theta1_Servo = Leg->theta1 + Leg->theta1_offset;
	Leg->theta2_Servo = Leg->theta2 + Leg->theta2_offset;
	Leg->theta3_Servo = Leg->theta3 + Leg->theta3_offset;
	//Update position
	Leg->target_x = x;
	Leg->target_y = y;
	Leg->target_z = z;
}

//Theta is degree
void InverseKinematics(Leg_t *Leg, float x, float y, float z){

	float L, x_23, y_23;
	float theta1, theta2, theta3, J3, A, B;
	if(y == 0){
		theta1 = 0;
	}
	else if(x == 0){
		theta1 = PI/2.0;
	}
	else
		theta1 = atan(y/x);
	//y1 = a1*sin(theta1);
	y_23 = y - a1*sin(theta1);
	x_23 = x - a1*cos(theta1);
	//float x_2 = x - a1;
	L = sqrtf(pow(x_23,2) + pow(y_23,2) + pow(z,2));
	J3 = acos((pow(a2,2) + pow(a3,2) - pow(L,2)) / (2*a2*a3));
	theta3 = J3 - PI;
	B = acos((pow(L,2) + pow(a2,2) - pow(a3,2)) / (2*L*a2));
	A = atan(-z/x_23);
	theta2 = B - A;
	Leg->theta1 = theta1 * 180/PI;
	Leg->theta2 = theta2 * 180/PI;
	Leg->theta3 = theta3 * 180/PI;

	switch (Leg->ID){
	    case 1:
	      // statements
	    	ControlServo(&Servo2, Leg->theta3 + Leg->theta3_offset);
			HAL_Delay(50);
			ControlServo(&Servo1, Leg->theta2 + Leg->theta2_offset);
			HAL_Delay(50);
			ControlServo(&Servo0, Leg->theta1+ Leg->theta1_offset);
			HAL_Delay(50);
	      break;

	    case 2:
	      // statements
	    	ControlServo(&Servo5, Leg->theta3 + Leg->theta3_offset);
			HAL_Delay(50);
			ControlServo(&Servo4, Leg->theta2 + Leg->theta2_offset);
			HAL_Delay(50);
			ControlServo(&Servo3, Leg->theta1+ Leg->theta1_offset);
			HAL_Delay(50);
	      break;

	    case 3:
		  // statements
	    	ControlServo(&Servo8, Leg->theta3 + Leg->theta3_offset);
			HAL_Delay(50);
	    	ControlServo(&Servo7, Leg->theta2 + Leg->theta2_offset);
	    	HAL_Delay(50);
			ControlServo(&Servo6, Leg->theta1+ Leg->theta1_offset);
			HAL_Delay(50);
		  break;

	    case 4:
		  // statements
	    	ControlServo(&Servo11, Leg->theta3 + Leg->theta3_offset);
			HAL_Delay(50);
			ControlServo(&Servo10, Leg->theta2 + Leg->theta2_offset);
			HAL_Delay(50);
			ControlServo(&Servo9, Leg->theta1+ Leg->theta1_offset);
			HAL_Delay(50);
		  break;
	    default:
	      // default statements
	}
	Leg->x = x;
	Leg->y = y;
	Leg->z = z;
}
//Theta is degree
void InverseDirect(Leg_t *Leg, float x, float y, float z){

	float L, x_23, y_23;
	float theta1, theta2, theta3, J3, A, B;
	if(y == 0){
		theta1 = 0;
	}
	else if(x == 0){
		theta1 = PI/2.0;
	}
	else
		theta1 = atan(y/x);
	//y1 = a1*sin(theta1);
	y_23 = y - a1*sin(theta1);
	x_23 = x - a1*cos(theta1);
	//float x_2 = x - a1;
	L = sqrtf(pow(x_23,2) + pow(y_23,2) + pow(z,2));
	J3 = acos((pow(a2,2) + pow(a3,2) - pow(L,2)) / (2*a2*a3));
	theta3 = J3 - PI;
	B = acos((pow(L,2) + pow(a2,2) - pow(a3,2)) / (2*L*a2));
	A = atan(-z/x_23);
	theta2 = B - A;
	//Convert theta to degree
	Leg->theta1 = theta1 * 180/PI;
	Leg->theta2 = theta2 * 180/PI;
	Leg->theta3 = theta3 * 180/PI;

	//Convert to servo angle
	Leg->theta1_Servo = Leg->theta1 + Leg->theta1_offset;
	Leg->theta2_Servo = Leg->theta2 + Leg->theta2_offset;
	Leg->theta3_Servo = Leg->theta3 + Leg->theta3_offset;
	//Update position
	Leg->target_x = x;
	Leg->target_y = y;
	Leg->target_z = z;

	switch (Leg->ID){
	    case 1:
	      // statements
	    	ControlServoDirect(&Servo2, Leg->theta3_Servo);
			//HAL_Delay(1);
			osDelay(1);
			ControlServoDirect(&Servo1, Leg->theta2_Servo);
			//HAL_Delay(1);
			osDelay(1);
			ControlServoDirect(&Servo0, Leg->theta1_Servo);
			//HAL_Delay(1);
			osDelay(1);
	      break;

	    case 2:
	      // statements
	    	ControlServoDirect(&Servo5, Leg->theta3 + Leg->theta3_offset);
			//HAL_Delay(1);
	    	osDelay(1);
			ControlServoDirect(&Servo4, Leg->theta2 + Leg->theta2_offset);
			//HAL_Delay(1);
			osDelay(1);
			ControlServoDirect(&Servo3, Leg->theta1 + Leg->theta1_offset);
			//HAL_Delay(1);
			osDelay(1);
	      break;

	    case 3:
		  // statements
	    	ControlServoDirect(&Servo8, Leg->theta3 + Leg->theta3_offset);
			//HAL_Delay(1);
	    	osDelay(1);
			ControlServoDirect(&Servo7, Leg->theta2 + Leg->theta2_offset);
	    	//HAL_Delay(1);
			osDelay(1);
	    	ControlServoDirect(&Servo6, Leg->theta1 + Leg->theta1_offset);
			//HAL_Delay(1);
	    	osDelay(1);
		  break;

	    case 4:
		  // statements
	    	ControlServoDirect(&Servo11, Leg->theta3 + Leg->theta3_offset);
			//HAL_Delay(1);
	    	osDelay(1);
			ControlServoDirect(&Servo10, Leg->theta2 + Leg->theta2_offset);
			//HAL_Delay(1);
			osDelay(1);
			ControlServoDirect(&Servo9, Leg->theta1 + Leg->theta1_offset);
			//HAL_Delay(1);
			osDelay(1);
		  break;
	    default:
	      // default statements
	}
	Leg->x = x;
	Leg->y = y;
	Leg->z = z;
}

void ControlLeg(Leg_t *Leg){
	if(Mode == 0){
		return;
	}
	float step[3];
	switch (Leg->ID){
		case 1:
		  // statements
			step[0] = (Leg->theta1_Servo - Servo0.Current_Angle)/MoveSteps;
			step[1] = (Leg->theta2_Servo - Servo1.Current_Angle)/MoveSteps;
			step[2] = (Leg->theta3_Servo - Servo2.Current_Angle)/MoveSteps;
			for(float i = 0; i < MoveSteps; i++){
				ControlServoDirect(&Servo2, Servo2.Current_Angle + step[2]);
				//HAL_Delay(T_control_leg);
				osDelay(T_control_leg);
				ControlServoDirect(&Servo1, Servo1.Current_Angle + step[1]);
				osDelay(T_control_leg);
				ControlServoDirect(&Servo0, Servo0.Current_Angle + step[0]);
				osDelay(T_control_leg);
			}
		  break;

		case 2:
		  // statements
			step[0] = (Leg->theta1_Servo - Servo3.Current_Angle)/MoveSteps;
			step[1] = (Leg->theta2_Servo - Servo4.Current_Angle)/MoveSteps;
			step[2] = (Leg->theta3_Servo - Servo5.Current_Angle)/MoveSteps;
			for(float i = 0; i < MoveSteps; i++){
				ControlServoDirect(&Servo5, Servo5.Current_Angle + step[2]);
				osDelay(T_control_leg);
				ControlServoDirect(&Servo4, Servo4.Current_Angle + step[1]);
				osDelay(T_control_leg);
				ControlServoDirect(&Servo3, Servo3.Current_Angle + step[0]);
				osDelay(T_control_leg);
			}
		  break;

		case 3:
		  // statements
			step[0] = (Leg->theta1_Servo - Servo6.Current_Angle)/MoveSteps;
			step[1] = (Leg->theta2_Servo - Servo7.Current_Angle)/MoveSteps;
			step[2] = (Leg->theta3_Servo - Servo8.Current_Angle)/MoveSteps;
			for(float i = 0; i < MoveSteps; i++){
				ControlServoDirect(&Servo8, Servo8.Current_Angle + step[2]);
				osDelay(T_control_leg);
				ControlServoDirect(&Servo7, Servo7.Current_Angle + step[1]);
				osDelay(T_control_leg);
				ControlServoDirect(&Servo6, Servo6.Current_Angle + step[0]);
				osDelay(T_control_leg);
			}
		  break;

		case 4:
		  // statements
			step[0] = (Leg->theta1_Servo - Servo9.Current_Angle)/MoveSteps;
			step[1] = (Leg->theta2_Servo - Servo10.Current_Angle)/MoveSteps;
			step[2] = (Leg->theta3_Servo - Servo11.Current_Angle)/MoveSteps;
			for(float i = 0; i < MoveSteps; i++){
				ControlServoDirect(&Servo11, Servo11.Current_Angle + step[2]);
				osDelay(T_control_leg);
				ControlServoDirect(&Servo10, Servo10.Current_Angle + step[1]);
				osDelay(T_control_leg);
				ControlServoDirect(&Servo9, Servo9.Current_Angle + step[0]);
				osDelay(T_control_leg);
			}
		  break;
		default:
		  // default statements
	}
	Leg->x = Leg->target_x;
	Leg->y = Leg->target_y;
	Leg->z = Leg->target_z;
}
//Roll: trước sau, Pitch: trái phải
void PID(float Roll_value, float Pitch_value, float Time) {
	if((Time <= 0) || (Time > 5000)){
		return;
	}
	Roll_E2 = Roll_E1;
	Roll_E1 = Roll_E0;
	Roll_E0 = Roll_Ref - Roll_value;
	Pitch_E2 = Pitch_E1;
	Pitch_E1 = Pitch_E0;
	Pitch_E0 = Pitch_Ref - Pitch_value;

	if(((Roll_value - Roll_Pre) > 40) || ((Roll_value - Roll_Pre) < -40) || ((Pitch_value - Pitch_Pre) < -40) || ((Pitch_value - Pitch_Pre) > 40)){
		return;
	}
	Roll_Pre = Roll_value;
	Pitch_Pre = Pitch_value;
//	Leg1_u1 = Leg1_u;
//	Leg2_u1 = Leg2_u;
//	Leg3_u1 = Leg3_u;
//	Leg4_u1 = Leg4_u;

	Roll_u1 = Roll_u;
	Pitch_u1 = Pitch_u;

	float a, b, g, delta;

	a = 2.0 * Time / 1000.0 * Kp + Ki * Time * Time / 1000.0 / 1000.0 + 2.0 * Kd;
	b = Time / 1000.0 * Time / 1000.0 * Ki - 4.0 * Kd - 2.0 * Time / 1000.0 * Kp;
	g = 2.0 * Kd;
	delta = 2.0 * Time / 1000.0;

//	a = 2 * T * kp + ki * T * T + 2 * kd;
//	b = T * T * ki - 4 * kd - 2 * T * kp;
//	g = 2 * kd;
//	a = 2 * T/1000 * kp + ki * T * T/1000/1000 + 2 * kd;
//	b = T/1000 * T/1000 * ki - 4 * kd - 2 * T/1000 * kp;
//	g = 2 * kd;
	//pwm = (a * e_current + b * E1 + g * E2 + delta * pwm_last) / delta;

	Roll_u = (a * Roll_E0 + b * Roll_E1 + g * Roll_E2 + delta * Roll_u1) / delta;
	Pitch_u = (a * Pitch_E0 + b * Pitch_E1 + g * Pitch_E2 + delta * Pitch_u1) / delta;

//	if (pwm - pwm_last < 50 && pwm - pwm_last > -50) {
//		pwm_last = pwm;
//		return pwm;
//	}

	//pwm = (a*e_current + b*E1 + g*E2 + 2*pwm_last)/2;

	//pwm = pwm_last + kp*(e_current - E1) + ki*T/2/1000*(e_current - E1)+ kd/T/1000*(e_current - 2*E1 + E2);

	//pwm = pwm_last + kp*(e_current - E1) + ki*T/2*(e_current + E1) + kd/T*(e_current - 2*E1 + E2);

	if (Roll_u > 80)
		Roll_u = 80;
	else if (Roll_u < -80)
		Roll_u = -80;

	if (Pitch_u > 80)
		Pitch_u = 80;
	else if (Pitch_u < -80)
		Pitch_u = -80;

}

float PID_Yaw(float Yaw_value, float Time) {
	if((Time <= 0) || (Time > 5000)){
		return 0;
	}
	Yaw_E2 = Yaw_E1;
	Yaw_E1 = Yaw_E0;
	Yaw_E0 = Yaw_Ref - Yaw_value;

//	if(((Yaw_value - Yaw_Pre) > 170) || ((Yaw_value - Yaw_Pre) < -40) || ((Pitch_value - Pitch_Pre) < -40) || ((Pitch_value - Pitch_Pre) > 40)){
//		return;
//	}
	if(Yaw_value - Yaw_Pre == 0){
		return 0;
	}
	Yaw_Pre = Yaw_value;

//	Leg1_u1 = Leg1_u;
//	Leg2_u1 = Leg2_u;
//	Leg3_u1 = Leg3_u;
//	Leg4_u1 = Leg4_u;

	Yaw_u1 = Yaw_u;

	float a, b, g, delta;

	a = 2.0 * Time / 1000.0 * Kp + Ki * Time * Time / 1000.0 / 1000.0 + 2.0 * Kd;
	b = Time / 1000.0 * Time / 1000.0 * Ki - 4.0 * Kd - 2.0 * Time / 1000.0 * Kp;
	g = 2.0 * Kd;
	delta = 2.0 * Time / 1000.0;

//	a = 2 * T * kp + ki * T * T + 2 * kd;
//	b = T * T * ki - 4 * kd - 2 * T * kp;
//	g = 2 * kd;
//	a = 2 * T/1000 * kp + ki * T * T/1000/1000 + 2 * kd;
//	b = T/1000 * T/1000 * ki - 4 * kd - 2 * T/1000 * kp;
//	g = 2 * kd;
	//pwm = (a * e_current + b * E1 + g * E2 + delta * pwm_last) / delta;

	Yaw_u = (a * Yaw_E0 + b * Yaw_E1 + g * Yaw_E2 + delta * Yaw_u1) / delta;

	if (Yaw_u > 50)
		Yaw_u = 50;
	else if (Yaw_u < -50)
		Yaw_u = -50;
	return Yaw_u;
}
//PCA9685_STATUS controlServo0(float Angle){
//	float Value;
//	if(Angle < 25) Angle = 25;
//	if(Angle > 155) Angle = 155;
//
//	//Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO0, 0, Value);
//}
//
//PCA9685_STATUS controlServo1(float Angle){
//	float Value;
//	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
//	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
//
//	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	//Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO1, 0, Value);
//}
//
//PCA9685_STATUS controlServo2(float Angle){
//	float Value;
//	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
//	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
//	//Thuận
//	//Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	//Nghịch
//	Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO2, 0, Value);
//}
//
//PCA9685_STATUS controlServo3(float Angle){
//	float Value;
//	if(Angle < 25) Angle = 25;
//	if(Angle > 155) Angle = 155;
//
//	//Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO3, 0, Value);
//}
//
//PCA9685_STATUS controlServo4(float Angle){
//	float Value;
//	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
//	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
//
//	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	//Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO4, 0, Value);
//}
//
//PCA9685_STATUS controlServo5(float Angle){
//	float Value;
//	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
//	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
//	//Thuận
//	//Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	//Nghịch
//	Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO5, 0, Value);
//}
//
//PCA9685_STATUS controlServo6(float Angle){
//	float Value;
//	if(Angle < 25) Angle = 25;
//	if(Angle > 155) Angle = 155;
//
//	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	//Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO6, 0, Value);
//}
//
//PCA9685_STATUS controlServo7(float Angle){
//	float Value;
//	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
//	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
//
//	//Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO7, 0, Value);
//}
//
//PCA9685_STATUS controlServo8(float Angle){
//	float Value;
//	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
//	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
//	//Thuận
//	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	//Nghịch
//	//Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO8, 0, Value);
//}
//
//PCA9685_STATUS controlServo9(float Angle){
//	float Value;
//	if(Angle < 25) Angle = 25;
//	if(Angle > 155) Angle = 155;
//
//	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	//Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO9, 0, Value);
//}
//
//PCA9685_STATUS controlServo10(float Angle){
//	float Value;
//	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
//	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
//
//	//Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO10, 0, Value);
//}
//
//PCA9685_STATUS controlServo11(float Angle){
//	float Value;
//	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
//	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
//	//Thuận
//	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
//	//Nghịch
//	//Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
//	return PCA9685_SetPwm(SERVO11, 0, Value);
//}
