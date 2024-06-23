/*
 * Servo.c
 *
 *  Created on: Mar 25, 2024
 *      Author: caoph
 */

#include "servo.h"
//#include "pca9685.h"

//extern osThreadId_t CommunicationHandle;
//#define Steps 40
#define T_servo 10
extern uint8_t Mode;
Servo_t Servo0 = {
		.ID = 0,
		.Min_Angle = 25,
		.Max_Angle = 155,
		.isForward = 0, //nghịch
		.Current_Angle = 95,
		.Angle_offset = 45
};
Servo_t Servo1 = {
		.ID = 1,
		.Min_Angle = 0,
		.Max_Angle = 180,
		.isForward = 1, //thuận
		.Current_Angle = 180,
		.Angle_offset = 70
};
Servo_t Servo2 = {
		.ID = 2,
		.Min_Angle = 0,
		.Max_Angle = 180,
		.isForward = 0, //nghịch
		.Current_Angle = 0,
		.Angle_offset = 138
};
Servo_t Servo3 = {
		.ID = 4,
		.Min_Angle = 25,
		.Max_Angle = 155,
		.isForward = 0, //nghịch
		.Current_Angle = 90,
		.Angle_offset = 141
};
Servo_t Servo4 = {
		.ID = 5,
		.Min_Angle = 0,
		.Max_Angle = 180,
		.isForward = 1, //thuận
		.Current_Angle = 180,
		.Angle_offset = 76
};
Servo_t Servo5 = {
		.ID = 6,
		.Min_Angle = 0,
		.Max_Angle = 180,
		.isForward = 0, //nghịch
		.Current_Angle = 0,
		.Angle_offset = 132
};
Servo_t Servo6 = {
		.ID = 8,
		.Min_Angle = 25,
		.Max_Angle = 255,
		.isForward = 1, //thuận
		.Current_Angle = 145,
		.Angle_offset = 134
};
Servo_t Servo7 = {
		.ID = 9,
		.Min_Angle = 0,
		.Max_Angle = 180,
		.isForward = 0, //nghịch
		.Current_Angle = 180,
		.Angle_offset = 77
};
Servo_t Servo8 = {
		.ID = 10,
		.Min_Angle = 0,
		.Max_Angle = 180,
		.isForward = 1, //thuận
		.Current_Angle = 0,
		.Angle_offset = 132
};
Servo_t Servo9 = {
		.ID = 13,
		.Min_Angle = 25,
		.Max_Angle = 155,
		.isForward = 1, //thuận
		.Current_Angle = 45,
		.Angle_offset = 43
};
Servo_t Servo10 = {
		.ID = 14,
		.Min_Angle = 0,
		.Max_Angle = 180,
		.isForward = 0, //nghịch
		.Current_Angle = 180,
		.Angle_offset = 74
};
Servo_t Servo11 = {
		.ID = 15,
		.Min_Angle = 0,
		.Max_Angle = 180,
		.isForward = 1, //thuận
		.Current_Angle = 0,
		.Angle_offset = 138
};


PCA9685_STATUS Set_Servo_Angle(uint8_t Servo_ID, uint8_t isForward, float Angle){

	float Value;
	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;
	if(isForward == 1)
		Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
	else
		Value = (-Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
	return PCA9685_SetPwm(Servo_ID, 0, Value);
}

PCA9685_STATUS ControlServoDirect(Servo_t *Servo, float Target_Angle){
	//osThreadSuspend(CommunicationHandle);
	if(Mode == 0){
		return 0;
	}
	if(Servo->Current_Angle == Target_Angle)
		return 0;
	float Value;
	//To observe inverse kinematics angle
	Servo->Target_Angle = Target_Angle;
	//Range: 0 - 180
	if(Target_Angle < Servo->Min_Angle){
		Target_Angle = Servo->Min_Angle;
	}
	else if(Target_Angle > Servo->Max_Angle){
		Target_Angle = Servo->Max_Angle;
	}
	//else Servo->Target_Angle = Target_Angle;

	if(Servo->isForward == 1)
		Value = (Target_Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
	else
		Value = (-Target_Angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MAX;
	//Range: 0 - 180
	Servo->Current_Angle = Target_Angle;
	//value is uint16_t
	//Value = (uint16_t)Value;
	//osThreadSuspend(CommunicationHandle);
	//PCA9685_SetPwm(Servo->ID, 0, Value);
	//osThreadResume(CommunicationHandle);
	return PCA9685_SetPwm(Servo->ID, 0, Value);
}
//Control servo with delay
void ControlServo(Servo_t *Servo, float Target_Angle){
	if(Mode == 0){
		return;
	}
	if(Servo->Current_Angle == Target_Angle)
		return;
	Servo->Target_Angle = Target_Angle;
	if(Target_Angle < Servo->Min_Angle){
		Servo->Target_Angle = Servo->Min_Angle;
	}
	else if(Target_Angle > Servo->Max_Angle){
		Servo->Target_Angle = Servo->Max_Angle;
	}
	//else Servo->Target_Angle = Target_Angle;

	if (Servo->Target_Angle > Servo->Current_Angle) {
		for (float angle = Servo->Current_Angle; angle <= Servo->Target_Angle; angle++) {
			Set_Servo_Angle(Servo->ID, Servo->isForward, angle);
			osDelay(T_servo);
			//HAL_Delay(15);
		}
	}
	else {
		for (float angle = Servo->Current_Angle; angle >= Servo->Target_Angle; angle--) {
			Set_Servo_Angle(Servo->ID, Servo->isForward, angle);
			osDelay(T_servo);
		}
	}
	Servo->Current_Angle = Servo->Target_Angle;
}

//void initServo() {
//	//Servo_t servo1;
//	//servo11
//	Servo_t Servo11;
//	Servo11.Min_PWM = 102; //xuống
//	Servo11.Max_PWM = 520;
//	Servo11.Min_Angle;
//	Servo11.Max_Angle;
//	Servo_t Servo10;
//	Servo10.Min_PWM = 102; //lên
//	Servo10.Max_PWM = 520;
//	Servo10.Min_Angle;
//	Servo10.Max_Angle;
//	Servo_t Servo9;
//	Servo9.Min_PWM = 102; //sau, chặn 161
//	Servo9.Max_PWM = 520; //trước, chặn 461
//	Servo9.Min_Angle;
//	Servo9.Max_Angle;
//	Servo_t Servo2;
//	Servo2.Min_PWM = 102; //lên
//	Servo2.Max_PWM = 520; //xuống
//	Servo2.Min_Angle;
//	Servo2.Max_Angle;
//	Servo_t Servo1;
//	Servo1.Min_PWM = 102; //xuống
//	Servo1.Max_PWM = 520; //
//	Servo1.Min_Angle;
//	Servo1.Max_Angle;
//	Servo_t Servo0;
//	Servo0.Min_PWM = 102; //trước, chặn 161 ->25.4 độ
//	Servo0.Max_PWM = 520; //sau, chặn 461 -> 154.6
//	Servo0.Min_Angle;
//	Servo0.Max_Angle;
//	Servo_t Servo5;
//	Servo5.Min_PWM = 102; //lên
//	Servo5.Max_PWM = 520; //
//	Servo5.Min_Angle;
//	Servo5.Max_Angle;
//	Servo_t Servo4;
//	Servo4.Min_PWM = 102; //xuống
//	Servo4.Max_PWM = 520; //
//	Servo4.Min_Angle;
//	Servo4.Max_Angle;
//	Servo_t Servo3;
//	Servo3.Min_PWM = 102; //trước, chặn 161
//	Servo3.Max_PWM = 520; //sau, chặn 461
//	Servo3.Min_Angle;
//	Servo3.Max_Angle;
//	Servo_t Servo8;
//	Servo8.Min_PWM = 102; //xuống
//	Servo8.Max_PWM = 520; //
//	Servo8.Min_Angle;
//	Servo8.Max_Angle;
//	Servo_t Servo7;
//	Servo7.Min_PWM = 102; //lên
//	Servo7.Max_PWM = 520; //
//	Servo7.Min_Angle;
//	Servo7.Max_Angle;
//	Servo_t Servo6;
//	Servo6.Min_PWM = 102; //sau, chặn 161
//	Servo6.Max_PWM = 520; //trước, chặn 461
//	Servo6.Min_Angle;
//	Servo6.Max_Angle;
//}

//void PCA_Set(uint8_t number, uint8_t startAngle, uint8_t endAngle, uint8_t mode,
//		uint8_t speed) {
//	uint8_t i;
//	uint32_t off = 0;
//	switch (mode) {
//	case 0: {
//		PCA9685_SetServoAngle(number, endAngle);
//
//	}
//		break;
//
//	case 1: {
//		PCA9685_SetServoAngle(number, endAngle);
//		if (endAngle > startAngle) {
//			HAL_Delay((uint16_t) ((endAngle - startAngle) * 2.7));
//		} else {
//			HAL_Delay((uint16_t) ((endAngle - startAngle) * 2.7));
//		}
//	}
//		break;
//	case 2: {
//		if (endAngle > startAngle) {
//			for (i = startAngle; i <= endAngle; i++) {
//				PCA9685_SetServoAngle(number, i);
//			}
//			HAL_Delay(2);
//			delay_us(speed * 250);
//		} else {
//			for (i = startAngle; i >= endAngle; i--) {
//				PCA9685_SetServoAngle(number, i);
//			}
//			HAL_Delay(2);
//			delay_us(speed * 250);
//		}
//	}
//		break;
//	}
//}

