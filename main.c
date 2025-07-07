/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t *channel;
    float maxAngleDeg;
    uint16_t minPulseUs;
    uint16_t maxPulseUs;
}Servo_t;

Servo_t servos[12] = {

	//{timer, channel, maxAngle, minPulse(us), maxPulse(us)}
    // --- Leg0 ---
    // Theta1 (1–2 ms)
    { &htim2, TIM_CHANNEL_1, 120.0f, 1000, 2000 },
    // Theta2 (1–2 ms)
    { &htim2, TIM_CHANNEL_2, 120.0f, 1000, 2000 },
    // Theta3 (0.5–2.5 ms)
    { &htim3, TIM_CHANNEL_1, 180.0f, 500, 2500 },

    // --- Leg1 ---
    { &htim3, TIM_CHANNEL_2, 120.0f, 1000, 2000 },
    { &htim3, TIM_CHANNEL_3, 120.0f, 1000, 2000 },
    { &htim3, TIM_CHANNEL_4, 180.0f, 500, 2500 },

    // --- Leg2 ---
    { &htim4, TIM_CHANNEL_1, 120.0f, 1000, 2000 },
    { &htim4, TIM_CHANNEL_2, 120.0f, 1000, 2000 },
    { &htim4, TIM_CHANNEL_3, 180.0f, 500, 2500 },

    // --- Leg3 ---
    { &htim4, TIM_CHANNEL_4, 120.0f, 1000, 2000 },
    { &htim1, TIM_CHANNEL_1, 120.0f, 1000, 2000 },
    { &htim1, TIM_CHANNEL_2, 180.0f, 500, 2500 },
};

#define D1 25
#define A2 95.1
#define A3 100.7

static uint16_t angleToPulse(float angleDeg, float maxAngle, uint16_t minPulse, uint16_t maxPulse){
	float pulse = (((float)(maxPulse - minPulse)/ maxAngle) * angleDeg)+((maxPulse+minPulse)/2);

	return (uint16_t)(pulse);


}

void write2Servo(uint8_t ServoNum, float angleDeg){
	Servo_t *s = &servos[ServoNum];
//	switch(ServoNum){
//		case 1:
//			angleDeg -= 10;
//			break;
//		case 4:
//			angleDeg += 10;
//			break;
//		case 7:
//			angleDeg += 10;
//			break;
//		case 10:
//			angleDeg -= 10;
//			break;
//	}
	uint16_t pulseUs = angleToPulse(angleDeg, s->maxAngleDeg, s->minPulseUs, s->maxPulseUs);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, pulseUs);
}

void writeSpeed(uint8_t ServoNum, float currAngle, float nextAngle, int delay){
	Servo_t *s = &servos[ServoNum];
	uint16_t currPulse = angleToPulse(currAngle, s->maxAngleDeg, s->minPulseUs, s->maxPulseUs);
	uint16_t nextPulse = angleToPulse(nextAngle, s->maxAngleDeg, s->minPulseUs, s->maxPulseUs);

	for(currPulse; currPulse < nextPulse; currPulse ++){
	    __HAL_TIM_SET_COMPARE(s->htim, s->channel, currPulse);
	    HAL_Delay(delay);

	}
}

#define DEFX 80
#define DEFY 0
#define DEFZ -146


  typedef struct {
	  float x;
	  float y;
	  float z;
  }Pos;

  typedef struct {
	  float theta1;
	  float theta2;
	  float theta3;
  }ServoAngles;

ServoAngles* inverseKinematics(Pos* position){
	 ServoAngles* angles = (ServoAngles*)malloc(sizeof(ServoAngles));
	 angles->theta1 = atan2(position->y, position->x);

	 float A = A2;
	 float B = A3;
	 float D = (position->x) * cos(angles->theta1) + (position->y) * sin(angles->theta1);
	 float H = (position->z);
	 float F = (pow(D, 2) + pow(H, 2) - pow(A, 2) - pow(B, 2))/(2*A);

	 angles->theta3 = (M_PI/2) - atan2(F, -sqrt(pow(B, 2)-pow(F, 2)));

	 float M = A + B*cos(angles->theta3);
	 float N = B*sin(angles->theta3);

	 float X = ((D*M) + (H*N))/(pow(M, 2) + pow(N, 2));
	 float Y = (H-(N*X))/M;

	 angles->theta2 = atan2(Y, X);




	 angles->theta1 = angles->theta1 * (180/M_PI);
	 angles->theta2 = angles->theta2 * (180/M_PI);

	 if ((angles->theta3 * (180/M_PI))<=180){
		 angles->theta3 = angles->theta3 * (180/M_PI);
	 }else{
		 angles->theta3 = (angles->theta3 * (180/M_PI))-(float)360;
	 }

	 return angles;
}

void legOne(int dir){
	Pos* position = (Pos*)malloc(sizeof(Pos));
	uint8_t d = D1;

	position -> x = DEFX;
	position -> y = DEFY;
	position -> z = DEFZ;

	ServoAngles* angles = (ServoAngles*)malloc(sizeof(Pos));

//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0); // mag on
//	HAL_Delay(250);

	//default
//	angles = inverseKinematics(position);
//
//	write2Servo(0, angles->theta1);
//	write2Servo(1, -angles->theta2);
//	write2Servo(2, -angles->theta3);
//	HAL_Delay(500);


	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1); // mag off
	HAL_Delay(2000);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	//raised
	position -> z = -90;
	position -> x = 110;
	angles = inverseKinematics(position);
	write2Servo(0, angles->theta1);
	write2Servo(1, -angles->theta2);
	write2Servo(2, -angles->theta3);
	HAL_Delay(500);

	//turn (DIR: 0->fwd, 1->right, 2->left)
	switch(dir){
		case 0:
			position -> x = DEFX + d*cos(M_PI/4);
			position -> y = -d*sin(M_PI/4);
			break;
		case 1:
			position -> x = -sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(M_PI/4)));
			position -> y = -d*sin(M_PI/4);
			break;
		case 2:
			position -> x = sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(3*M_PI/4)));
			position -> y = d*sin(M_PI/4);
			break;
	}

	angles = inverseKinematics(position);
	write2Servo(0, angles->theta1);
	write2Servo(1, -angles->theta2);
	write2Servo(2, -angles->theta3);
	HAL_Delay(500);

	//down

	position -> z = DEFZ;
	angles = inverseKinematics(position);
	write2Servo(0, angles->theta1);
	write2Servo(1, -angles->theta2);
	write2Servo(2, -angles->theta3);
//	writeSpeed(2, -prev_theta3, -angles->theta3, 15);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0); // mag on
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_Delay(50);

//	//back to default (TO BE CHECKED)
//	position -> x = DEFX;
//	position -> y = DEFY;
//	angles = inverseKinematics(position);
//	write2Servo(0, angles->theta1);
//	write2Servo(1, -angles->theta2);
//	write2Servo(2, -angles->theta3);
//	HAL_Delay(500);

}

void legTwo(int dir){
	Pos* position = (Pos*)malloc(sizeof(Pos));
	uint8_t d = D1;

	//default
	position -> x = DEFX;
	position -> y = DEFY;
	position -> z = DEFZ;

	ServoAngles* angles = (ServoAngles*)malloc(sizeof(Pos));

//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // mag on


//	HAL_Delay(250);

//	angles = inverseKinematics(position);
//	write2Servo(3, angles->theta1);
//	write2Servo(4, angles->theta2);
//	write2Servo(5, angles->theta3);
//	HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1); // mag off
	HAL_Delay(2000);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	//raised
	position -> z = -90;
	position -> x = 110;
	angles = inverseKinematics(position);
	write2Servo(3, angles->theta1);
	write2Servo(4, angles->theta2);
	write2Servo(5, angles->theta3);
	HAL_Delay(500);

	//turn (DIR: 0->fwd, 1->right, 2->left)
	switch(dir){
		case 0:
			position -> x = DEFX + d*cos(M_PI/4);
			position -> y = d*sin(M_PI/4);
			break;
		case 1:
			position -> x = sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(3*M_PI/4)));
			position -> y = -d*sin(M_PI/4);
			break;
		case 2:
			position -> x = -sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(M_PI/4)));
			position -> y = d*sin(M_PI/4);
			break;
	}

	angles = inverseKinematics(position);
	write2Servo(3, angles->theta1);
	write2Servo(4, angles->theta2);
	write2Servo(5, angles->theta3);
	HAL_Delay(500);

	//down

	position -> z = DEFZ;
	angles = inverseKinematics(position);
	write2Servo(3, angles->theta1);
	write2Servo(4, angles->theta2);
	write2Servo(5, angles->theta3);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // mag on
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_Delay(50);


//	//back to default (TO BE CHECKED)
//	position -> x = DEFX;
//	position -> y = DEFY;
//	angles = inverseKinematics(position);
//	write2Servo(3, angles->theta1);
//	write2Servo(4, angles->theta2);
//	write2Servo(5, angles->theta3);
//	HAL_Delay(1000);
}

void legThree(int dir){
	Pos* position = (Pos*)malloc(sizeof(Pos));
	uint8_t d = D1;

	//default
	position -> x = DEFX;
	position -> y = DEFY;
	position -> z = DEFZ;

	ServoAngles* angles = (ServoAngles*)malloc(sizeof(Pos));

//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // mag on
//
//	HAL_Delay(250);

//	angles = inverseKinematics(position);
//	write2Servo(6, angles->theta1);
//	write2Servo(7, angles->theta2);
//	write2Servo(8, angles->theta3);
//	HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); // mag off
	HAL_Delay(2000);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	//raised
	position -> z = -90;
	position -> x = 110;

	angles = inverseKinematics(position);
	write2Servo(6, angles->theta1);
	write2Servo(7, angles->theta2);
	write2Servo(8, angles->theta3);
	HAL_Delay(500);


	//turn (DIR: 0->fwd, 1->right, 2->left)
	switch(dir){
		case 0:
			position -> x = DEFX - d*cos(M_PI/4);;
			position -> y = d*sin(M_PI/4);
			break;
		case 1:
			position -> x = sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(3*M_PI/4)));
			position -> y = d*sin(M_PI/4);
			break;
		case 2:
			position -> x = -sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(M_PI/4)));
			position -> y = -d*sin(M_PI/4);
			break;
	}
	angles = inverseKinematics(position);
	write2Servo(6, angles->theta1);
	write2Servo(7, angles->theta2);
	write2Servo(8, angles->theta3);
	HAL_Delay(500);


	//down

	position -> z = DEFZ;
	angles = inverseKinematics(position);
	write2Servo(6, angles->theta1);
	write2Servo(7, angles->theta2);
	write2Servo(8, angles->theta3);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // mag on
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_Delay(50);


//	//back to default (TO BE CHECKED)
//	position -> x = DEFX;
//	position -> y = DEFY;
//	angles = inverseKinematics(position);
//	write2Servo(6, angles->theta1);
//	write2Servo(7, angles->theta2);
//	write2Servo(8, angles->theta3);
//	HAL_Delay(1000);

}

void legFour(int dir) {
	Pos* position = (Pos*)malloc(sizeof(Pos));
	uint8_t d = D1;

	//default
	position -> x = DEFX;
	position -> y = DEFY;
	position -> z = DEFZ;

	ServoAngles* angles = (ServoAngles*)malloc(sizeof(Pos));

//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // mag on
//	HAL_Delay(250);

//	angles = inverseKinematics(position);
//	write2Servo(9, angles->theta1);
//	write2Servo(10, -angles->theta2);
//	write2Servo(11, -angles->theta3);
//	HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // mag off
	HAL_Delay(2000);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


	//raised
	position -> z = -90;
	position -> x = 110;

	angles = inverseKinematics(position);
	write2Servo(9, angles->theta1);
	write2Servo(10, -angles->theta2);
	write2Servo(11, -angles->theta3);
	HAL_Delay(500);


	//turn (DIR: 0->fwd, 1->right, 2->left)
	switch(dir){
		case 0:
			position -> x = DEFX - d*cos(M_PI/4);
			position -> y = -d*sin(M_PI/4);
			break;
		case 1:
			position -> x = -sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(M_PI/4)));
			position -> y = d*sin(M_PI/4);
			break;
		case 2:
			position -> x = sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(3*M_PI/4)));
			position -> y = -d*sin(M_PI/4);
			break;
	}

	angles = inverseKinematics(position);
	write2Servo(9, angles->theta1);
	write2Servo(10, -angles->theta2);
	write2Servo(11, -angles->theta3);
	HAL_Delay(500);


	//down

	position -> z = DEFZ;
	angles = inverseKinematics(position);
	write2Servo(9, angles->theta1);
	write2Servo(10,-angles->theta2);
	write2Servo(11,-angles->theta3);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0); // mag on
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	HAL_Delay(50);


//	//back to default (TO BE CHECKED)
//	position -> x = DEFX;
//	position -> y = DEFY;
//	angles = inverseKinematics(position);
//	write2Servo(9, angles->theta1);
//	write2Servo(10, -angles->theta2);
//	write2Servo(11, -angles->theta3);
//	HAL_Delay(1000);


}

void setDefaultPos(){
	Pos* position = (Pos*)malloc(sizeof(Pos));
	uint8_t d = D1;

	position -> x = DEFX;
	position -> y = DEFY;
	position -> z = DEFZ;

	ServoAngles* angles = (ServoAngles*)malloc(sizeof(Pos));

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0); // mag on
	HAL_Delay(50);

	//default
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	angles = inverseKinematics(position);
	write2Servo(0, angles->theta1);
	write2Servo(1, -angles->theta2);
	write2Servo(2, -angles->theta3);

	write2Servo(3, angles->theta1);
	write2Servo(4, angles->theta2);
	write2Servo(5, angles->theta3);

	write2Servo(6, angles->theta1);
	write2Servo(7, angles->theta2);
	write2Servo(8, angles->theta3);

	write2Servo(9, angles->theta1);
	write2Servo(10, -angles->theta2);
	write2Servo(11, -angles->theta3);

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);


	HAL_Delay(500);
}

//void forwardOne(){
//
//	Pos* position = (Pos*)malloc(sizeof(Pos));
//	uint8_t d = 30;
//
//	//default
//	position -> x = DEFX;
//	position -> y = DEFY;
//	position -> z = DEFZ;
//
//	ServoAngles* angles = (ServoAngles*)malloc(sizeof(Pos));
//
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0); // mag on
//	HAL_Delay(50);
//
//	angles = inverseKinematics(position);
//	write2Servo(0, angles->theta1);
//	write2Servo(1, angles->theta2);
//	write2Servo(2, angles->theta3);
//	HAL_Delay(500);
//
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1); // mag off
//	HAL_Delay(50);
//
//	//raised
//	position -> z = -50;
//	angles = inverseKinematics(position);
//	write2Servo(0, angles->theta1);
//	write2Servo(1, angles->theta2);
//	write2Servo(2, angles->theta3);
//	HAL_Delay(500);
//
//
//	//turn
//	position -> x = sqrt(pow(d,2) + pow(position->x, 2) - (2*d*(position->x)*cos(3*M_PI/4)));
//	position -> y = -d*sin(M_PI/4);
//	angles = inverseKinematics(position);
//	write2Servo(0, angles->theta1);
//	write2Servo(1, angles->theta2);
//	write2Servo(2, angles->theta3);
//	HAL_Delay(500);
//
//
//	//down
//	position -> z = DEFZ;
//	angles = inverseKinematics(position);
//	write2Servo(0, angles->theta1);
//	write2Servo(1, angles->theta2);
//	write2Servo(2, angles->theta3);
//	HAL_Delay(2000);
//
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0); // mag on
//	HAL_Delay(50);
//
//	//back to default (TO BE CHECKED)
//	position -> x = DEFX;
//	position -> y = DEFY;
//	angles = inverseKinematics(position);
//	write2Servo(0, angles->theta1);
//	write2Servo(1, angles->theta2);
//	write2Servo(2, angles->theta3);
//	HAL_Delay(500);
//
//
//	int debug = 0;
//
//
//
//
//}

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1){


}

while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1){


}

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);



HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1500);

//
//int x = 5;
//
HAL_Delay(5000);
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);

__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angleToPulse(-30, 120, 1000, 2000)); //motor joint 2
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, angleToPulse(0, 120, 1000, 2000)); //motor joint 1
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, angleToPulse(0, 180, 500, 2500)); //motor joint 3

__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, angleToPulse(30, 120, 1000, 2000));
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, angleToPulse(0, 120, 1000, 2000));
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, angleToPulse(0, 180, 500, 2500));

__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, angleToPulse(30, 120, 1000, 2000));
__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, angleToPulse(0, 120, 1000, 2000));
__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angleToPulse(0, 180, 500, 2500));

__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, angleToPulse(-30, 120, 1000, 2000));
__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, angleToPulse(0, 120, 1000, 2000));
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, angleToPulse(0, 180, 500, 2500));
HAL_Delay(2000);
//
//
//
//
//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, angleToPulse(70, 180, 500, 2500));
//
//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, angleToPulse(-70, 180, 500, 2500));
//
//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angleToPulse(-70, 180, 500, 2500));
//
//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, angleToPulse(70, 180, 500, 2500));
//
//
//HAL_Delay(5000); //change -60 to -50 for curved to flat
//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, angleToPulse(30, 120, 1000, 2000));
//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, angleToPulse(50, 180, 500, 2500));
//
//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, angleToPulse(-30, 120, 1000, 2000));
//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, angleToPulse(-50, 180, 500, 2500));
//
//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, angleToPulse(-30, 120, 1000, 2000));
//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angleToPulse(-50, 180, 500, 2500));
//
//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, angleToPulse(30, 120, 1000, 2000));
//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, angleToPulse(50, 180, 500, 2500));
//
//HAL_Delay(2500);
//
//forwardOne();
//forwardTwo();
//forwardThree();
//forwardFour();
setDefaultPos();
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
HAL_Delay(8000);
legOne(0);
legTwo(0);
legThree(0);
legFour(0);
setDefaultPos();

HAL_Delay(500);
legOne(0);
legTwo(0);
legThree(0);
legFour(0);
setDefaultPos();

HAL_Delay(500);
legOne(0);
legTwo(0);
legThree(0);
legFour(0);
setDefaultPos();

HAL_Delay(25000);

HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);








  while (1)
  {
//	  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1){
//
//
//	  }
//
//	  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1){
//
//
//	  }
//	  legOne(0);
//	  legTwo(0);
//	  legThree(0);
//	  legFour(0);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
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
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  while (1)
  {
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
