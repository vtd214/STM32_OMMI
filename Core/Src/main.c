/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PWM Test 4 Motors - STM32F401 + BTS7960
  *
  * Motor mapping:
  *   Motor 1 = Front Left  = Banh truoc trai
  *   Motor 2 = Front Right = Banh truoc phai
  *   Motor 3 = Rear Left   = Banh sau trai
  *   Motor 4 = Rear Right  = Banh sau phai
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  const char *name;

  TIM_HandleTypeDef *lpwm_htim;
  uint32_t lpwm_channel;

  TIM_HandleTypeDef *rpwm_htim;
  uint32_t rpwm_channel;

  GPIO_TypeDef *en_port;
  uint16_t en_pin;

  int motor_sign;
  int current_pwm;

} MotorPWM_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOTOR_COUNT       4

#define MOTOR_NORMAL      1
#define MOTOR_INVERT     -1

/*
  Neu Counter Period = 999:
    300 = 30%
    600 = 60%
    900 = 90%
*/
#define PWM_LOW           300
#define PWM_MEDIUM        600
#define PWM_HIGH          900

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
  Motor 1 = Front Left = banh truoc trai
    LPWM PA8  -> TIM1_CH1
    RPWM PA9  -> TIM1_CH2
    EN   PC13

  Motor 2 = Front Right = banh truoc phai
    LPWM PA10 -> TIM1_CH3
    RPWM PA11 -> TIM1_CH4
    EN   PC14

  Motor 3 = Rear Left = banh sau trai
    LPWM PA2 -> TIM9_CH1
    RPWM PA3 -> TIM9_CH2
    EN   PC15

  Motor 4 = Rear Right = banh sau phai
    LPWM PB8 -> TIM10_CH1
    RPWM PB9 -> TIM11_CH1
    EN   PB12
*/

MotorPWM_t motors[MOTOR_COUNT] =
{
  {
    "M1_FRONT_LEFT",
    &htim1, TIM_CHANNEL_1,
    &htim1, TIM_CHANNEL_2,
    GPIOC, GPIO_PIN_13,
    MOTOR_NORMAL,
    0
  },

  {
    "M2_FRONT_RIGHT",
    &htim1, TIM_CHANNEL_3,
    &htim1, TIM_CHANNEL_4,
    GPIOC, GPIO_PIN_14,
    MOTOR_NORMAL,
    0
  },

  {
    "M3_REAR_LEFT",
    &htim9, TIM_CHANNEL_1,
    &htim9, TIM_CHANNEL_2,
    GPIOC, GPIO_PIN_15,
    MOTOR_NORMAL,
    0
  },

  {
    "M4_REAR_RIGHT",
    &htim10, TIM_CHANNEL_1,
    &htim11, TIM_CHANNEL_1,
    GPIOB, GPIO_PIN_12,
    MOTOR_NORMAL,
    0
  }
};

/*
  Debug variables.
  Xem trong Live Expressions.
*/
volatile int debug_pwm_m1 = 0;
volatile int debug_pwm_m2 = 0;
volatile int debug_pwm_m3 = 0;
volatile int debug_pwm_m4 = 0;

volatile int debug_test_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

void Motor_Enable(MotorPWM_t *motor);
void Motor_Disable(MotorPWM_t *motor);
void Motor_PWM_Start(MotorPWM_t *motor);
void Motor_SetPWM(MotorPWM_t *motor, int pwm);
void Motor_Stop(MotorPWM_t *motor);

void Motors_PWM_InitAll(void);
void Motors_SetPWMAll(int pwm);
void Motors_StopAll(void);
void Motors_UpdateDebugVariables(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Motor_Enable(MotorPWM_t *motor)
{
  HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);
}

void Motor_Disable(MotorPWM_t *motor)
{
  HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_RESET);
}

void Motor_PWM_Start(MotorPWM_t *motor)
{
  HAL_TIM_PWM_Start(motor->lpwm_htim, motor->lpwm_channel);
  HAL_TIM_PWM_Start(motor->rpwm_htim, motor->rpwm_channel);

  Motor_Enable(motor);
  Motor_Stop(motor);
}

void Motor_SetPWM(MotorPWM_t *motor, int pwm)
{
  uint32_t lpwm_max = __HAL_TIM_GET_AUTORELOAD(motor->lpwm_htim);
  uint32_t rpwm_max = __HAL_TIM_GET_AUTORELOAD(motor->rpwm_htim);

  uint32_t pwm_max = lpwm_max;

  if (rpwm_max < pwm_max)
  {
    pwm_max = rpwm_max;
  }

  /*
    Dao chieu motor bang software neu can.
    Neu motor nao quay nguoc so voi mong muon,
    doi motor_sign cua motor do thanh MOTOR_INVERT.
  */
  pwm = pwm * motor->motor_sign;

  if (pwm > (int)pwm_max)
  {
    pwm = pwm_max;
  }

  if (pwm < -(int)pwm_max)
  {
    pwm = -pwm_max;
  }

  Motor_Enable(motor);

  motor->current_pwm = pwm;

  if (pwm > 0)
  {
    /*
      Quay thuan:
      LPWM co PWM
      RPWM = 0
    */
    __HAL_TIM_SET_COMPARE(motor->lpwm_htim, motor->lpwm_channel, pwm);
    __HAL_TIM_SET_COMPARE(motor->rpwm_htim, motor->rpwm_channel, 0);
  }
  else if (pwm < 0)
  {
    /*
      Quay nguoc:
      LPWM = 0
      RPWM co PWM
    */
    __HAL_TIM_SET_COMPARE(motor->lpwm_htim, motor->lpwm_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->rpwm_htim, motor->rpwm_channel, -pwm);
  }
  else
  {
    Motor_Stop(motor);
  }
}

void Motor_Stop(MotorPWM_t *motor)
{
  /*
    Stop an toan:
    LPWM = 0
    RPWM = 0
    Tat EN de tranh BTS7960 ham cung motor.
  */
  __HAL_TIM_SET_COMPARE(motor->lpwm_htim, motor->lpwm_channel, 0);
  __HAL_TIM_SET_COMPARE(motor->rpwm_htim, motor->rpwm_channel, 0);

  motor->current_pwm = 0;

  Motor_Disable(motor);
}

void Motors_PWM_InitAll(void)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_PWM_Start(&motors[i]);
  }

  Motors_StopAll();
  Motors_UpdateDebugVariables();
}

void Motors_SetPWMAll(int pwm)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_SetPWM(&motors[i], pwm);
  }

  Motors_UpdateDebugVariables();
}

void Motors_StopAll(void)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_Stop(&motors[i]);
  }

  Motors_UpdateDebugVariables();
}

void Motors_UpdateDebugVariables(void)
{
  debug_pwm_m1 = motors[0].current_pwm;
  debug_pwm_m2 = motors[1].current_pwm;
  debug_pwm_m3 = motors[2].current_pwm;
  debug_pwm_m4 = motors[3].current_pwm;
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();

  /*
    PWM timers:
      TIM1  = Motor 1 + Motor 2
      TIM9  = Motor 3
      TIM10 = Motor 4 LPWM
      TIM11 = Motor 4 RPWM
  */
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();

  /* USER CODE BEGIN 2 */

  Motors_PWM_InitAll();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
      Pha 1: ca 4 motor quay thuan PWM 300
    */
    debug_test_state = 1;
    Motors_SetPWMAll(PWM_LOW);
    HAL_Delay(3000);

    /*
      Dung
    */
    debug_test_state = 2;
    Motors_StopAll();
    HAL_Delay(1000);

    /*
      Pha 2: ca 4 motor quay thuan PWM 600
    */
    debug_test_state = 3;
    Motors_SetPWMAll(PWM_MEDIUM);
    HAL_Delay(3000);

    /*
      Dung
    */
    debug_test_state = 4;
    Motors_StopAll();
    HAL_Delay(1000);

    /*
      Pha 3: ca 4 motor quay thuan PWM 900
    */
    debug_test_state = 5;
    Motors_SetPWMAll(PWM_HIGH);
    HAL_Delay(3000);

    /*
      Dung
    */
    debug_test_state = 6;
    Motors_StopAll();
    HAL_Delay(1000);

    /*
      Pha 4: ca 4 motor quay nguoc PWM 600
    */
    debug_test_state = 7;
    Motors_SetPWMAll(-PWM_MEDIUM);
    HAL_Delay(3000);

    /*
      Dung
    */
    debug_test_state = 8;
    Motors_StopAll();
    HAL_Delay(2000);

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  __disable_irq();

  while (1)
  {
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
