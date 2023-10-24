/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

char send[20];

uint8_t value[16];
int LastLineError, PreviousLineError, PreviousyError, PreviousbalanceError;
float Line_PID_Output, Line_PID_I;
float LMotorSpeed;
float RMotorSpeed;
float motorSpeed = 43;
float cornerspeed = 50;
int junction_counter = 0;
int junction = 0;
int counter = 1;
int max_counter;
int status;

int expected_junction;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* Ultrasonic variables */
#define TRIG_PIN GPIO_PIN_15
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_12
#define ECHO_PORT GPIOA
uint32_t pMillis;
float Value1 = 0;
float Value2 = 0;
int Distance = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void motor_stop(void) {

//RIGHT MOTOR
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

	//LEFT MOTOR
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);

}

int line_error(void) {

	int sensor_readings = 0;
	int weighted_output = 0;
	int error = 0;

	for (int i = 0; i < 8; i++) {

		sensor_readings = sensor_readings + value[2 * i];
	}

	weighted_output = weighted_output + (-15 * value[0]);
//sprintf(send, "A1 = %d ",  value[0]);
//HAL_UART_Transmit(&huart1, send, strlen(send), 100);

	weighted_output = weighted_output + (-3 * value[2]);

int Data=what variable you want to send
sprintf(send, "A2 = %d ",  Data);
HAL_UART_Transmit(&huart2, send, strlen(send), 100);


	weighted_output = weighted_output + (-2 * value[4]);
//sprintf(send, "A3 = %d ",  value[4]);
//HAL_UART_Transmit(&huart1, send, strlen(send), 100);

	weighted_output = weighted_output + (-1 * value[6]);
//sprintf(send, "A4 = %d ",  value[6]);
//HAL_UART_Transmit(&huart1, send, strlen(send), 100);

	weighted_output = weighted_output + (1 * value[8]);
//sprintf(send, "A5 = %d ",  value[8]);
//HAL_UART_Transmit(&huart1, send, strlen(send), 100);

	weighted_output = weighted_output + (2 * value[10]);
//sprintf(send, "A6 = %d ",  value[10]);
//HAL_UART_Transmit(&huart1, send, strlen(send), 100);

	weighted_output = weighted_output + (3 * value[12]);
//sprintf(send, "A7 = %d ",  value[12]);
//HAL_UART_Transmit(&huart1, send, strlen(send), 100);

	weighted_output = weighted_output + (15 * value[14]);
//sprintf(send, "A8 = %d ",  value[14]);
//HAL_UART_Transmit(&huart1, send, strlen(send), 100);
//HAL_UART_Transmit(&huart1, "\r\n", 3, 100);

	error = weighted_output / sensor_readings;

//find junction
	if ((value[0] > 180) && (value[2] > 180) && (value[4] > 180)
			&& (value[6] > 180) && (value[8] > 180) && (value[10] > 180)
			&& (value[12] > 180) && (value[14] > 180) && (status == 0)
			&& (counter > +max_counter)) {

		junction = junction + 1;
		counter = 0;
		HAL_Delay(70);
		motor_stop();
		counter = 0;
		status = 1;

	}

	if ((value[0] > 180) && (value[2] > 180) && (value[4] > 180)
			&& (value[6] > 180) && (status == 0) && (counter >= max_counter)
			&& (expected_junction == 1)) {

		//left_corner = left_corner + 1;

		//HAL_Delay(200);

		junction = junction + 1;
		motor_stop();
		HAL_Delay(50);
		counter = 0;
		status = 1;

	}

	if ((value[8] > 180) && (value[10] > 180) && (value[12] > 180)
			&& (value[14] > 180) && (status == 0) && (counter >= max_counter)
			&& (expected_junction == 2)) {

		//right_corner = right_corner + 1;

		//HAL_Delay(200);
		junction = junction + 1;

		motor_stop();
		HAL_Delay(50);
		status = 1;

	}

	status = 0;
	return error;
}

void motor_left_init(void) {

	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

}

void motor_left_speed(int rpm) {
	int y = 100 * rpm;

	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, y);

}

void motor_left_reverse(void) {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);

}

void motor_left_forward(void) {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0);

}

void motor_left_mover(int direction, int speed) {
//0 forward 1 reverse
	switch (direction) {
	case 0:
		motor_left_forward();
		break;

	case 1:
		motor_left_reverse();
		break;

	}

	motor_left_speed(speed);

}

void motor_right_init(void) {

	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

}

void motor_right_speed(int rpm) {
	int y = 100 * rpm;

	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, y);

}

void motor_right_forward(void) {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);

}

void motor_right_reverse(void) {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

}

void motor_right_mover(int direction, int speed) {

	switch (direction) {
	case 0:
		motor_right_forward();
		break;

	case 1:
		motor_right_reverse();
		break;

	}

	motor_right_speed(speed);

}

void line_PID(int line_error) {

	float Line_PID_P, Line_PID_D;
	float Line_Kp = 25, Line_Ki = 0, Line_Kd = 35; //10

	Line_PID_P = line_error;
	Line_PID_I = Line_PID_I + line_error;
	Line_PID_D = (line_error - PreviousLineError);
	Line_PID_Output = (Line_PID_P * Line_Kp) + (Line_PID_I * Line_Ki)
			+ (Line_Kd * Line_PID_D);

	if (Line_PID_Output < -200)
		Line_PID_Output = -200;
	if (Line_PID_Output > 200)
		Line_PID_Output = 200;
	PreviousLineError = line_error;

}

void sonar_Init(void) {
	HAL_TIM_Base_Start(&htim1);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low
}

int sonar(void) {
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < 10)
		;  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	// wait for the echo pin to go high
	while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
			&& pMillis + 10 > HAL_GetTick())
		;
	Value1 = __HAL_TIM_GET_COUNTER(&htim1);

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
			&& pMillis + 50 > HAL_GetTick())
		;
	Value2 = __HAL_TIM_GET_COUNTER(&htim1);

	Distance = ((Value2 - Value1) * 0.034 / 2);
	return Distance;
}

void turn_Crazyleft(void) {
	motor_right_mover(0, motorSpeed);
	motor_left_mover(0, motorSpeed);
	HAL_Delay((2.5 * motorSpeed));
	motor_stop();
	int z = 0;

	while ((value[0] > 10) || (value[2] > 10) || (value[4] > 10)
			|| (value[6] > 10) || (value[8] > 10) || (value[10] > 10)
			|| (value[12] > 10) || (value[14] > 10)) {

		motor_right_mover(0, cornerspeed);
		motor_left_mover(1, cornerspeed);

		if (z < 1) {
			HAL_Delay(450);
		}
		if (z >= 100) {
			z = 3;
		}
		z++;
		//			sprintf(send, "Turning 1 = %d\r\n", error_1);
		//			HAL_UART_Transmit(&huart1, send, sizeof(send), 100);
		if ((value[0] > 180) || (value[2] > 180)) {
			motor_stop();
			HAL_Delay(300);
			junction = junction + 1;
			break;

		}
	}

}

void turn_Crazyright(void) {

	motor_right_mover(0, motorSpeed);
	motor_left_mover(0, motorSpeed);
	HAL_Delay((2.5 * motorSpeed));
	motor_stop();
	int z = 0;

	while ((value[0] > 10) || (value[2] > 10) || (value[4] > 10)
			|| (value[6] > 10) || (value[8] > 10) || (value[10] > 10)
			|| (value[12] > 10) || (value[14] > 10)) {

		motor_right_mover(1, cornerspeed);
		motor_left_mover(0, cornerspeed);

		if (z < 1) {
			HAL_Delay(450);
		}
		if (z >= 100) {
			z = 3;
		}
		z++;
		//			sprintf(send, "Turning 1 = %d\r\n", error_1);
		//			HAL_UART_Transmit(&huart1, send, sizeof(send), 100);
		if ((value[14] > 180) || (value[12] > 180)) {
			motor_stop();
			HAL_Delay(300);
			junction = junction + 1;
			break;
		}
	}

}

void about_turn(void) {/*Rightside about turn*/
	motor_stop();
	int z = 0;

	while ((value[0] > 10) || (value[2] > 10) || (value[4] > 10)
			|| (value[6] > 10) || (value[8] > 10) || (value[10] > 10)
			|| (value[12] > 10) || (value[14] > 10)) {

		motor_right_mover(1, cornerspeed);
		motor_left_mover(0, cornerspeed);

		if (z < 1) {
			HAL_Delay(330);
		}
		if (z >= 100) {
			z = 3;
		}
		z++;
		//			sprintf(send, "Turning 1 = %d\r\n", error_1);
		//			HAL_UART_Transmit(&huart1, send, sizeof(send), 100);
		if ((value[14] > 180) || (value[12] > 180)) {
			motor_stop();
			HAL_Delay(300);
			junction = junction + 1;
			break;
		}
	}

}



void Center_to_pick(void) {
	while (line_error() != 0) {
		int error1 = line_error();
		if (error1 > 0) {
			motor_right_mover(0, 44);
			motor_left_mover(1, 44);
		}
		if (error1 < 0) {
			motor_right_mover(1, 44);
			motor_left_mover(0, 44);

		}
	}
//	if ((value[0] > 180) || (value[2] > 180)) {
//		while (((value[6] + value[8]) < 450)) {
//			motor_right_mover(0, 44);
//			motor_left_mover(1, 44);
//		}
//
//	} else if ((value[12] > 180) || (value[14] > 180)) {
//		while (((value[6] + value[8]) < 450)) {
//			motor_right_mover(1, 44);
//			motor_left_mover(0, 44);
//		}
//
//	}

	motor_stop();
	HAL_Delay(500);
}

void Crazy_about_turn() {
	motor_stop();
	int z = 0;
	int turns = 0;

	while ((value[0] > 10) || (value[2] > 10) || (value[4] > 10)
			|| (value[6] > 10) || (value[8] > 10) || (value[10] > 10)
			|| (value[12] > 10) || (value[14] > 10)) {

		motor_right_mover(1, cornerspeed);
		motor_left_mover(0, cornerspeed);

		if (z < 1) {
			HAL_Delay(500);
		}
		if (z >= 100) {
			z = 3;
		}
		z++;
		//			sprintf(send, "Turning 1 = %d\r\n", error_1);
		//			HAL_UART_Transmit(&huart1, send, sizeof(send), 100);
		if ((value[14] > 180) || (value[12] > 180)) {
			motor_stop();
			HAL_Delay(300);
			Center_to_pick();
			turns = 2;
			z=0;

		}

		if (((value[14] + value[12]) > 360) && (turns == 2)) {
			motor_stop();
			HAL_Delay(300);
			junction = junction + 1;
			break;

		}
	}

}

void Accurate_distance(int Distance) {

	if (sonar() < Distance) {
		while (sonar() < Distance) {
			motor_right_mover(0, 40);
			motor_left_mover(0, 40);
		}
	} else if (sonar() > Distance) {
		while (sonar() > Distance) {
			motor_right_mover(1, 40);
			motor_left_mover(1, 40);
		}

	}
	motor_stop();
	HAL_Delay(500);
}

void servo_init() {
	HAL_TIM_PWM_Init(&htim2);
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_Init(&htim4);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void gripper(int angle) {
	int y = 11.11 * angle + 500;

//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, y);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, y);
}

void wrist(int angle) {
	int y = 11.11 * angle + 500;

	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, y);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, y);
}

void elbow(int angle) {
	int y = 11.11 * angle + 500;

	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, y);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, y);
}

void shoulder(int angle) {
	int y = 11.11 * angle + 500;

	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, y);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, y);
}

void base(int angle) {
	int y = 11.11 * angle + 500;

	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, y);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, y);
}

void open_gripper(void) {
	gripper(0);
}

void close_gripper(void) {
	for (int i = 0; i < 50; i++) {
		gripper(i);
		HAL_Delay(25);
	}
}

void HOME_POSITION_B4(void) {
	open_gripper();
	base(10);
	shoulder(135);
	elbow(0);
	wrist(70);
}

void HOME_POSITION_AFTER() {
	open_gripper();
	base(10);
	shoulder(160);
	elbow(0);
	wrist(100);
}
void Engine_Pickup(void) {
	open_gripper();
	for (int i = 0; i < 180; i++) {
		wrist(70 - (i * 0.388888888889));
		elbow(0 + (i * 0.4444));
		shoulder(135 - (i * 0.6944444444));
		HAL_Delay(25);
	}
	close_gripper();
	for (int i = 180; i > -1; i--) {
		wrist(70 - (i * 0.388888888889));
		elbow(0 + (i * 0.4444));
		shoulder(135 - (i * 0.69444444444));
		HAL_Delay(25);
	}
}

void Engine_Drop(void) {
	for (int i = 0; i < 180; i++) {
		base(10);
		shoulder(135 - (0.6944444 * i));
		elbow(0 + (i * 0.8333333333333333));
		wrist(70);
		HAL_Delay(25);
	}
	open_gripper();
	for (int i = 180; i > -1; i--) {
		base(10);
		shoulder(135 - (0.69444444 * i));
		elbow(0 + (i * 0.8333333333333333));
		wrist(70);
		HAL_Delay(25);
	}
}

void All_Wheels_Pickup(void) {
	open_gripper();
	for (int i = 0; i < 180; i++) {
		wrist(70 - (i * 0.3888888889));
		elbow(0 + (i * 0.4444));
		shoulder(135 - (i * 0.7333333333333333));
		HAL_Delay(10);
	}
	close_gripper();
	for (int i = 10; i < 70; i++) {
		base(i);
		HAL_Delay(10);
	}
	for (int i = 0; i < 140; ++i) {
		wrist(i * 0.714285714);
		elbow(80 - (i * 0.285714285));
		shoulder(3 + (i * 1.05));
		HAL_Delay(10);
	}
	for (int i = 70; i < 180; ++i) {
		base(i);
		HAL_Delay(10);
	}
	open_gripper(); // Drop the 1st wheel
	for (int i = 180; i > 9; --i) {
		base(i);
		//shoulder(150 - (0.088235294));
		HAL_Delay(10);
	}
	for (int i = 0; i < 120; ++i) {
		shoulder(150 - (i));
		elbow(40 + (i * 0.25));
		wrist(100 - (i * 0.75));
		HAL_Delay(10);
	}
	close_gripper(); // Pick the second wheel
	for (int i = 10; i < 70; i++) {
		base(i);
		HAL_Delay(10);
	}
	for (int i = 120; i > -1; --i) {
		shoulder(150 - (i));
		elbow(40 + (i * 0.25));
		wrist(100 - (i * 0.75));
		HAL_Delay(10);
	}
	for (int i = 70; i < 180; ++i) {
		base(i);
		HAL_Delay(10);
	}
	open_gripper(); //drop the second wheels
	for (int i = 180; i > 9; --i) {
		base(i);
		HAL_Delay(10);
	}
	for (int i = 0; i < 90; ++i) {
		shoulder(150 - (i * 1.0223333333333333));
		elbow(40 + (i * 0.2222222222222222));
		wrist(100 - (i * 0.8));
		HAL_Delay(10);
	}
	close_gripper(); //pick the third wheel elbow
	for (int i = 10; i < 70; i++) {
		base(i);
		HAL_Delay(10);
	}
	for (int i = 90; i > -1; --i) {
		shoulder(150 - i);
		elbow(40 + (i * 0.11111111));
		wrist(100 - (i * 0.93333333));
		HAL_Delay(10);
	}
	for (int i = 70; i < 180; ++i) {
		base(i);
		HAL_Delay(10);
	}
	open_gripper(); //drop the third wheel
//	pick the top wheel
//	****************************************
	for (int i = 180; i > 9; --i) {
		base(i);
		HAL_Delay(10);
	}
	for (int i = 0; i < 80; ++i) {
		shoulder(150 - (i * 1.025));
		elbow(40 + (0.5 * i));
		wrist(100 - (0.5625 * i));
		HAL_Delay(10);
	}
	close_gripper(); //GRIP/PICK THE TOP WHEEL wrist
	for (int i = 80; i > -1; --i) {
		shoulder(150 - (i * 1.025));
		elbow(40 + (0.5 * i));
		wrist(100 - (0.5625 * i));
		HAL_Delay(10);
	}
	for (int i = 10; i < 180; ++i) {
		base(i);
		HAL_Delay(10);
	}
	open_gripper(); //DROP THE TOP WHEEL
//	***************************************************
//	PICK THE CONTAINER WITH ALL THE WHEELS
	for (int i = 0; i < 20; ++i) {
		shoulder(150 + i);
		elbow(40 - i);
		wrist(100 + (i * 0.25));
		HAL_Delay(10);
	}
	close_gripper();
}

void All_Wheels_Drop(void) {
//	*************************************
	close_gripper();
	for (int i = 0; i < 25; ++i) {
		shoulder(170 - (i * 0.8));
		elbow(20 + i);
		HAL_Delay(10);
	}
	for (int i = 0; i < 140; ++i) {
		base(180 - (i * 0.2142857142857143));
		shoulder(150 - i);
		elbow(45 + (i * 0.75));
		wrist(105 - (i * 0.1428571429));
		HAL_Delay(10);
	}
	open_gripper();
	for (int i = 0; i < 150; ++i) {
		base(150 - (i));
		shoulder(10 + i);
		elbow(150 - i);
		wrist(85 + (i * 0.1));
		HAL_Delay(10);
	}
//	shoulder(170);
//	elbow(20);
//	wrist(105);
//	close_gripper();
//	shoulder(150);
//	elbow(45);
}

void Cabin_Pickup(void) {
	for (int i = 0; i < 150; ++i) {
		base(10);
		shoulder(160 - i);
		elbow(i * 0.5333333333333333);
		wrist(100 - (i * 0.6666666666666667));
		HAL_Delay(10);
	}
	close_gripper();
	for (int i = 150; i > -1; --i) {
		base(10);
		shoulder(160 - i);
		elbow(i * 0.5333333333333333);
		wrist(100 - (i * 0.6666666666666667));
		HAL_Delay(10);
	}
	for (int i = 0; i < 100; ++i) {
		wrist(100 - i);
		HAL_Delay(10);
	}
}

void Drop_Wheels_2(void) {
	//HOME_POSITION()
	base(180);
	shoulder(150);
	elbow(40);
	wrist(100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) value, 8);
	servo_init();
	sonar_Init();

	motor_left_init();
	motor_right_init();

	HOME_POSITION_B4();
	max_counter = 15;
	int Doonce = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		int error = line_error();
		line_PID(error);

		LMotorSpeed = motorSpeed + Line_PID_Output;
		RMotorSpeed = motorSpeed - Line_PID_Output;

		if (LMotorSpeed > 200)
			LMotorSpeed = 200;
		if (LMotorSpeed < 0)
			LMotorSpeed = 0;
		if (RMotorSpeed > 200)
			RMotorSpeed = 200;
		if (RMotorSpeed < 0)
			RMotorSpeed = 0;

		motor_left_mover(0, (int) LMotorSpeed);
		motor_right_mover(0, (int) RMotorSpeed);

		counter = counter + 1;

		/*BLUETOOTH MODULE TELEMETRY UART DATA TRANSMITTING*/
//		sprintf(send, "J = %d ", junction);
//		HAL_UART_Transmit(&huart1, send, strlen(send), 100);
//		HAL_UART_Transmit(&huart1, "\r\n", 3, 100);
		/*JUNCTION STORY LINE BEGIN*/
		/*after each junction JUNCTION=JUNCTION+1 ,and with every turn(left,right about-turn) JUNCTION=JUNCTION+1*/

		if (junction == 1) {/*JUNCTION 1*/
			max_counter = 30;	//prevent pre mature junction trigger
			motor_stop();
			HAL_Delay(50);
			expected_junction = 2;	//Allows Right L-junction to trigger
			junction = 2;
			motorSpeed = 40;
		}

		if (junction == 3) {/*JUNCTION 2*/
			motor_stop();
			HAL_Delay(100);
			max_counter = 150;	//prevents premature junction trigger
			expected_junction = 0;	//prevents false trigger for L-junctions
			turn_Crazyright();
			motor_stop();
			HAL_Delay(1000);
			motorSpeed = 40;
		}

		if (junction == 6) {/*JUNCTION 4*/
			motorSpeed = 35;
			int distance = sonar();

			if (distance <= 26) {
				motor_stop();
				HAL_Delay(1000);
				Center_to_pick();
				//Accurate_distance(25);
				HAL_Delay(1000);
				Engine_Pickup();

				HAL_Delay(300);
				motor_right_mover(1, 65);
				motor_left_mover(1, 65);
				HAL_Delay(100);
				motor_stop();
				about_turn();
				junction = 7;
				max_counter = 80;
				motorSpeed = 45;

			}

		}

		if (junction == 8) {/*JUNCTION 5*/
			motor_stop();
			HAL_Delay(300);
			turn_Crazyright();
			max_counter = 20;
			motorSpeed = 35;

		}

		if (junction == 10) {/*JUNCTION 6*/
			motor_stop();
			HAL_Delay(500);
			turn_Crazyleft();
			max_counter = 50;
			motorSpeed = (0.8 * 45);
		}

		if (junction == 11) {
			int distance = sonar();

			if (distance <= 9) {
				motor_stop();
				HAL_Delay(1000);
				Center_to_pick();
				//Accurate_distance(6);
				HAL_Delay(1000);
				Engine_Drop();
				HAL_Delay(1000);
				motor_right_mover(1, 65);
				motor_left_mover(1, 65);
				HAL_Delay(150);
				about_turn();

			}
		}

		if (junction == 13) {/*JUNCTION 7 towards wheel rack*/
			int distance = sonar();/*Ultrasonic distance ranging*/

			if ((distance <= 40) && (Doonce < 1)) {
				motorSpeed = 34;

				Center_to_pick();
				Doonce = 2;
			}
			if (distance <= 25) {
				motor_stop();
				HAL_Delay(1000);
				Center_to_pick();
				//Accurate_distance(25);
				HAL_Delay(1000);
				All_Wheels_Pickup();
				HAL_Delay(300);
				motor_right_mover(1, 65);
				motor_left_mover(1, 63);
				HAL_Delay(70);
				about_turn();
			}
		}

		if (junction == 15) {/*JUNCTION 8*/
			motor_stop();
			HAL_Delay(50);
			turn_Crazyright();
			expected_junction = 1;
			max_counter = 10;
		}

		if (junction == 17) {/*JUNCTION 9*/
			motorSpeed = 45;
			motor_stop();
			HAL_Delay(200);
			turn_Crazyleft();
			expected_junction = 0;
			max_counter = 80;
		}

		if (junction == 19) {
			motor_stop();
			HAL_Delay(100);
			All_Wheels_Drop();
			HAL_Delay(1000);
			turn_Crazyright();
			expected_junction = 0;
			max_counter = 50;
		}

		if (junction == 21) {
			motor_stop();
			HAL_Delay(50);
			turn_Crazyright();
			expected_junction = 0;
			max_counter = 50;
		}

		if (junction == 23) {

			motorSpeed = 100;
			max_counter = 20;

		}
		if (junction == 24) {

			if (sonar() <= 70) {
				motorSpeed = 36;
			}
		}

		if (junction == 25) {
			motor_stop();
			HAL_Delay(1000);
			motorSpeed = 36;
			int distance = sonar();/*Get the Cabin*/
			if (distance <= 20) {
				motor_stop();
				HAL_Delay(100);
				Center_to_pick();
				//Accurate_distance(6);
				HAL_Delay(500);
				Cabin_Pickup();
				HAL_Delay(300);

				while (sonar() > 3) {
					int error = line_error();
					line_PID(error);

					LMotorSpeed = motorSpeed + Line_PID_Output;
					RMotorSpeed = motorSpeed - Line_PID_Output;

					if (LMotorSpeed > 200)
						LMotorSpeed = 200;
					if (LMotorSpeed < 0)
						LMotorSpeed = 0;
					if (RMotorSpeed > 200)
						RMotorSpeed = 200;
					if (RMotorSpeed < 0)
						RMotorSpeed = 0;

					motor_left_mover(0, (int) LMotorSpeed);
					motor_right_mover(0, (int) RMotorSpeed);
				}

				motor_stop();
				HAL_Delay(300);
				about_turn();
				HAL_Delay(1000);
				motorSpeed = 32;

			}
		}

		if (junction == 30) {
			motorSpeed = 35;
			motor_stop();
			HAL_Delay(50);
			turn_Crazyleft();
			expected_junction = 0;
			max_counter = 20;
		}

		if (junction == 32) {
			motor_stop();
			HAL_Delay(50);
			turn_Crazyright();
			expected_junction = 0;
			max_counter = 80;
		}

		if (junction == 34) {
			motor_stop();
			HAL_Delay(50);
			turn_Crazyleft();
			max_counter = 10;
		}

		if (junction == 36) {
			motor_stop();
			HAL_Delay(50);
			turn_Crazyleft();
			HAL_Delay(50);
			motorSpeed = 45;

		}

		if (junction == 37) {/*drop the cabin*/
			int distance = sonar();/*Takes the cabin to the chasis*/
			if (distance <= 8) {
				motor_stop();
				HAL_Delay(1000);
				Center_to_pick();
				//Accurate_distance(5);
				Engine_Drop();
				HAL_Delay(300);
				motor_right_mover(1, 65);
				motor_left_mover(1, 65);
				HAL_Delay(100);
				about_turn();
				expected_junction = 2;
				max_counter = 80;
			}
		}

		if (junction == 39) {
			expected_junction = 1;
			max_counter = 60;
		}

		if (junction == 40) {
			motor_stop();
			HAL_Delay(50);
			turn_Crazyleft();
			HAL_Delay(50);
			expected_junction = 1;
			max_counter = 30;
		}

		if (junction == 42) {
			expected_junction = 0;
			max_counter = 10;
		}

		if (junction == 43) {

			motor_right_mover(0, 40);
			motor_left_mover(0, 40);
			HAL_Delay(300);
			motor_stop();
			junction = 44;
		}

		if (junction == 44) {
			HAL_Delay(100);
			motor_stop();

		}

	}/*end while*/

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
