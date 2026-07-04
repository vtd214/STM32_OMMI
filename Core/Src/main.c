/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PID Speed Control 4 Motors - Target riêng từng bánh
  *                   STM32F401CCU6 + BTS7960 + Encoder
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
  float kp;
  float ki;
  float kd;

  float integral;
  float prev_error;
  float output;

} PID_t;

typedef struct
{
  const char *name;

  TIM_HandleTypeDef *lpwm_htim;
  uint32_t lpwm_channel;

  TIM_HandleTypeDef *rpwm_htim;
  uint32_t rpwm_channel;

  GPIO_TypeDef *en_port;
  uint16_t en_pin;

  TIM_HandleTypeDef *encoder_htim;

  int motor_sign;
  int encoder_sign;

  int current_pwm;

  uint32_t encoder_last_count;
  int32_t encoder_delta;
  int64_t encoder_total_count;

  float speed_mps;
  float target_speed_mps;

  PID_t pid;

} Motor_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOTOR_COUNT                 4

#define MOTOR_NORMAL                1
#define MOTOR_INVERT               -1

#define ENCODER_NORMAL              1
#define ENCODER_INVERT             -1

#define PWM_MAX_VALUE               999
#define PWM_MIN_RUN                 100

#define CONTROL_PERIOD_MS           10
#define CONTROL_PERIOD_S            ((float)CONTROL_PERIOD_MS / 1000.0f)

/*
  Thông số bánh + encoder.
  Nếu motor của bạn là hộp số 1:90 thì đổi GEAR_RATIO thành 90.0f.
*/
#define WHEEL_DIAMETER_M            0.100f
#define PI_VALUE                    3.1415926f
#define WHEEL_CIRCUMFERENCE_M       (PI_VALUE * WHEEL_DIAMETER_M)

#define ENCODER_PPR                 11.0f
#define GEAR_RATIO                  30.0f
#define ENCODER_MULTIPLIER          4.0f

#define ENCODER_COUNT_PER_REV       (ENCODER_PPR * GEAR_RATIO * ENCODER_MULTIPLIER)
#define METER_PER_COUNT             (WHEEL_CIRCUMFERENCE_M / ENCODER_COUNT_PER_REV)

/*
  PID ban đầu.
  Nếu motor bị giật/lắc mạnh: giảm KP xuống 80, KI xuống 20.
  Nếu motor lên tốc độ quá chậm: tăng KP từ từ.
*/
#define PID_KP                      120.0f
#define PID_KI                      60.0f
#define PID_KD                      0.0f

#define PID_INTEGRAL_LIMIT          5.0f

/*
  Feedforward: ước lượng PWM theo tốc độ mong muốn.
  Từ test trước: khoảng PWM 300 -> ~0.8 m/s, PWM 600 -> ~1.7 m/s.
*/
#define PWM_PER_MPS                 340.0f

#define TARGET_DEADBAND_MPS         0.02f
#define TARGET_SPEED_LIMIT_MPS      2.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
  Mapping motor hiện tại:

  Motor 1 = Front Left = bánh trước trái
    LPWM    PA8  -> TIM1_CH1
    RPWM    PA9  -> TIM1_CH2
    EN      PC13
    Encoder TIM2

  Motor 2 = Front Right = bánh trước phải
    LPWM    PA10 -> TIM1_CH3
    RPWM    PA11 -> TIM1_CH4
    EN      PC14
    Encoder TIM3

  Motor 3 = Rear Left = bánh sau trái
    LPWM    PA2 -> TIM9_CH1
    RPWM    PA3 -> TIM9_CH2
    EN      PC15
    Encoder TIM4

  Motor 4 = Rear Right = bánh sau phải
    LPWM    PB8 -> TIM10_CH1
    RPWM    PB9 -> TIM11_CH1
    EN      PB12
    Encoder TIM5
*/

/* ===== DEBUG VARIABLES - thêm vào Live Expressions ===== */

volatile int debug_pwm_m1 = 0;
volatile int debug_pwm_m2 = 0;
volatile int debug_pwm_m3 = 0;
volatile int debug_pwm_m4 = 0;

volatile int32_t debug_delta_m1 = 0;
volatile int32_t debug_delta_m2 = 0;
volatile int32_t debug_delta_m3 = 0;
volatile int32_t debug_delta_m4 = 0;

volatile int64_t debug_total_m1 = 0;
volatile int64_t debug_total_m2 = 0;
volatile int64_t debug_total_m3 = 0;
volatile int64_t debug_total_m4 = 0;

volatile float debug_speed_m1 = 0.0f;
volatile float debug_speed_m2 = 0.0f;
volatile float debug_speed_m3 = 0.0f;
volatile float debug_speed_m4 = 0.0f;

volatile float debug_target_m1 = 0.0f;
volatile float debug_target_m2 = 0.0f;
volatile float debug_target_m3 = 0.0f;
volatile float debug_target_m4 = 0.0f;

volatile float debug_error_m1 = 0.0f;
volatile float debug_error_m2 = 0.0f;
volatile float debug_error_m3 = 0.0f;
volatile float debug_error_m4 = 0.0f;

volatile float debug_pid_m1 = 0.0f;
volatile float debug_pid_m2 = 0.0f;
volatile float debug_pid_m3 = 0.0f;
volatile float debug_pid_m4 = 0.0f;

volatile int debug_test_state = 0;

/* ===== MOTOR OBJECTS ===== */

Motor_t motors[MOTOR_COUNT] =
{
  {
    "M1_FRONT_LEFT",
    &htim1, TIM_CHANNEL_1,
    &htim1, TIM_CHANNEL_2,
    GPIOC, GPIO_PIN_13,
    &htim2,
    MOTOR_NORMAL,
    ENCODER_INVERT,
    0,
    0, 0, 0,
    0.0f,
    0.0f,
    {PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, 0.0f}
  },

  {
    "M2_FRONT_RIGHT",
    &htim1, TIM_CHANNEL_3,
    &htim1, TIM_CHANNEL_4,
    GPIOC, GPIO_PIN_14,
    &htim3,
    MOTOR_NORMAL,
    ENCODER_INVERT,
    0,
    0, 0, 0,
    0.0f,
    0.0f,
    {PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, 0.0f}
  },

  {
    "M3_REAR_LEFT",
    &htim9, TIM_CHANNEL_1,
    &htim9, TIM_CHANNEL_2,
    GPIOC, GPIO_PIN_15,
    &htim4,
    MOTOR_NORMAL,
    ENCODER_INVERT,
    0,
    0, 0, 0,
    0.0f,
    0.0f,
    {PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, 0.0f}
  },

  {
    "M4_REAR_RIGHT",
    &htim10, TIM_CHANNEL_1,
    &htim11, TIM_CHANNEL_1,
    GPIOB, GPIO_PIN_12,
    &htim5,
    MOTOR_NORMAL,
    ENCODER_INVERT,
    0,
    0, 0, 0,
    0.0f,
    0.0f,
    {PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, 0.0f}
  }
};

uint32_t last_control_time = 0;
uint32_t last_motion_time = 0;
uint8_t motion_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* Helper */
static float ClampFloat(float value, float min_value, float max_value);
static int ClampInt(int value, int min_value, int max_value);

/* Motor PWM */
void Motor_Enable(Motor_t *motor);
void Motor_Disable(Motor_t *motor);
void Motor_PWM_Start(Motor_t *motor);
void Motor_SetPWM(Motor_t *motor, int pwm);
void Motor_Stop(Motor_t *motor);

void Motors_PWM_InitAll(void);
void Motors_StopAll(void);

/* Encoder */
void Encoder_Reset(Motor_t *motor);
void Encoders_InitAll(void);
int32_t Encoder_ReadDelta(Motor_t *motor);
float Encoder_DeltaToMPS(int32_t delta);
void Encoders_UpdateAll(void);

/* PID */
void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float target, float feedback, float dt);
void Motor_SetTargetSpeed(Motor_t *motor, float target_speed_mps);
void Motors_SetTargetSpeedAll(float target_speed_mps);
void Motors_SetTargetSpeeds(float m1, float m2, float m3, float m4);
void Motors_ControlUpdateAll(void);

/* Debug */
void Debug_UpdateVariables(void);

/* Motion test */
void Motion_Speed_TestTask(void);

/* App */
void App_Init(void);
void App_Task(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float ClampFloat(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}

static int ClampInt(int value, int min_value, int max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}

/* =========================
   MOTOR PWM FUNCTIONS
   ========================= */

void Motor_Enable(Motor_t *motor)
{
  HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);
}

void Motor_Disable(Motor_t *motor)
{
  HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_RESET);
}

void Motor_PWM_Start(Motor_t *motor)
{
  HAL_TIM_PWM_Start(motor->lpwm_htim, motor->lpwm_channel);
  HAL_TIM_PWM_Start(motor->rpwm_htim, motor->rpwm_channel);

  Motor_Enable(motor);
  Motor_Stop(motor);
}

void Motor_SetPWM(Motor_t *motor, int pwm)
{
  uint32_t lpwm_max = __HAL_TIM_GET_AUTORELOAD(motor->lpwm_htim);
  uint32_t rpwm_max = __HAL_TIM_GET_AUTORELOAD(motor->rpwm_htim);

  uint32_t pwm_max = lpwm_max;

  if (rpwm_max < pwm_max)
  {
    pwm_max = rpwm_max;
  }

  if (pwm_max > PWM_MAX_VALUE)
  {
    pwm_max = PWM_MAX_VALUE;
  }

  /*
    Đảo chiều motor bằng software nếu cần.
    Nếu motor nào quay ngược so với mong muốn,
    đổi motor_sign của motor đó thành MOTOR_INVERT.
  */
  pwm = pwm * motor->motor_sign;

  pwm = ClampInt(pwm, -(int)pwm_max, (int)pwm_max);

  Motor_Enable(motor);

  motor->current_pwm = pwm;

  if (pwm > 0)
  {
    __HAL_TIM_SET_COMPARE(motor->lpwm_htim, motor->lpwm_channel, (uint32_t)pwm);
    __HAL_TIM_SET_COMPARE(motor->rpwm_htim, motor->rpwm_channel, 0);
  }
  else if (pwm < 0)
  {
    __HAL_TIM_SET_COMPARE(motor->lpwm_htim, motor->lpwm_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->rpwm_htim, motor->rpwm_channel, (uint32_t)(-pwm));
  }
  else
  {
    Motor_Stop(motor);
  }
}

void Motor_Stop(Motor_t *motor)
{
  __HAL_TIM_SET_COMPARE(motor->lpwm_htim, motor->lpwm_channel, 0);
  __HAL_TIM_SET_COMPARE(motor->rpwm_htim, motor->rpwm_channel, 0);

  motor->current_pwm = 0;

  /*
    Stop kiểu thả trôi hơn là phanh cứng:
    LPWM = 0, RPWM = 0, EN = 0.
  */
  Motor_Disable(motor);
}

void Motors_PWM_InitAll(void)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_PWM_Start(&motors[i]);
  }

  Motors_StopAll();
}

void Motors_StopAll(void)
{
  Motors_SetTargetSpeedAll(0.0f);

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_Stop(&motors[i]);
  }
}

/* =========================
   ENCODER FUNCTIONS
   ========================= */

void Encoder_Reset(Motor_t *motor)
{
  __HAL_TIM_SET_COUNTER(motor->encoder_htim, 0);

  motor->encoder_last_count = 0;
  motor->encoder_delta = 0;
  motor->encoder_total_count = 0;
  motor->speed_mps = 0.0f;
}

void Encoders_InitAll(void)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    HAL_TIM_Encoder_Start(motors[i].encoder_htim, TIM_CHANNEL_ALL);
    Encoder_Reset(&motors[i]);
  }
}

int32_t Encoder_ReadDelta(Motor_t *motor)
{
  uint32_t now_count = __HAL_TIM_GET_COUNTER(motor->encoder_htim);
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->encoder_htim);

  int64_t delta = (int64_t)now_count - (int64_t)motor->encoder_last_count;
  uint64_t period = (uint64_t)arr + 1ULL;

  if (delta > (int64_t)(period / 2ULL))
  {
    delta -= (int64_t)period;
  }
  else if (delta < -(int64_t)(period / 2ULL))
  {
    delta += (int64_t)period;
  }

  /*
    Trước đó encoder của bạn đang ra âm khi chạy thuận,
    nên mặc định để ENCODER_INVERT cho cả 4 bánh.
  */
  delta = delta * motor->encoder_sign;

  motor->encoder_last_count = now_count;
  motor->encoder_delta = (int32_t)delta;
  motor->encoder_total_count += delta;

  return motor->encoder_delta;
}

float Encoder_DeltaToMPS(int32_t delta)
{
  float distance_m = (float)delta * METER_PER_COUNT;
  float speed_mps = distance_m / CONTROL_PERIOD_S;

  return speed_mps;
}

void Encoders_UpdateAll(void)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    int32_t delta = Encoder_ReadDelta(&motors[i]);
    motors[i].speed_mps = Encoder_DeltaToMPS(delta);
  }
}

/* =========================
   PID FUNCTIONS
   ========================= */

void PID_Reset(PID_t *pid)
{
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output = 0.0f;
}

float PID_Update(PID_t *pid, float target, float feedback, float dt)
{
  float error = target - feedback;

  pid->integral += error * dt;
  pid->integral = ClampFloat(pid->integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

  float derivative = (error - pid->prev_error) / dt;

  pid->output = (pid->kp * error)
              + (pid->ki * pid->integral)
              + (pid->kd * derivative);

  pid->prev_error = error;

  return pid->output;
}

void Motor_SetTargetSpeed(Motor_t *motor, float target_speed_mps)
{
  target_speed_mps = ClampFloat(target_speed_mps,
                                -TARGET_SPEED_LIMIT_MPS,
                                TARGET_SPEED_LIMIT_MPS);

  /*
    Khi đổi chiều hoặc target = 0 thì reset PID để tránh tích phân cũ.
  */
  if ((motor->target_speed_mps > 0.0f && target_speed_mps < 0.0f) ||
      (motor->target_speed_mps < 0.0f && target_speed_mps > 0.0f) ||
      (target_speed_mps < TARGET_DEADBAND_MPS && target_speed_mps > -TARGET_DEADBAND_MPS))
  {
    PID_Reset(&motor->pid);
  }

  motor->target_speed_mps = target_speed_mps;
}

void Motors_SetTargetSpeedAll(float target_speed_mps)
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_SetTargetSpeed(&motors[i], target_speed_mps);
  }
}

/*
  Hàm quan trọng mới:
  set target speed riêng cho từng bánh.

  m1 = bánh trước trái
  m2 = bánh trước phải
  m3 = bánh sau trái
  m4 = bánh sau phải
*/
void Motors_SetTargetSpeeds(float m1, float m2, float m3, float m4)
{
  Motor_SetTargetSpeed(&motors[0], m1);
  Motor_SetTargetSpeed(&motors[1], m2);
  Motor_SetTargetSpeed(&motors[2], m3);
  Motor_SetTargetSpeed(&motors[3], m4);

  debug_target_m1 = motors[0].target_speed_mps;
  debug_target_m2 = motors[1].target_speed_mps;
  debug_target_m3 = motors[2].target_speed_mps;
  debug_target_m4 = motors[3].target_speed_mps;
}

void Motors_ControlUpdateAll(void)
{
  Encoders_UpdateAll();

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_t *motor = &motors[i];

    if (motor->target_speed_mps < TARGET_DEADBAND_MPS &&
        motor->target_speed_mps > -TARGET_DEADBAND_MPS)
    {
      PID_Reset(&motor->pid);
      Motor_Stop(motor);
      continue;
    }

    float pid_output = PID_Update(&motor->pid,
                                  motor->target_speed_mps,
                                  motor->speed_mps,
                                  CONTROL_PERIOD_S);

    /*
      Feedforward + PID correction.
    */
    float pwm_float = (motor->target_speed_mps * PWM_PER_MPS) + pid_output;

    int pwm_cmd = (int)pwm_float;

    /*
      Bù ma sát tĩnh: nếu target khác 0 mà PWM quá nhỏ,
      nâng lên PWM_MIN_RUN để motor không bị ì.
    */
    if (pwm_cmd > 0 && pwm_cmd < PWM_MIN_RUN)
    {
      pwm_cmd = PWM_MIN_RUN;
    }
    else if (pwm_cmd < 0 && pwm_cmd > -PWM_MIN_RUN)
    {
      pwm_cmd = -PWM_MIN_RUN;
    }

    pwm_cmd = ClampInt(pwm_cmd, -PWM_MAX_VALUE, PWM_MAX_VALUE);

    Motor_SetPWM(motor, pwm_cmd);
  }
}

/* =========================
   DEBUG FUNCTION
   ========================= */

void Debug_UpdateVariables(void)
{
  debug_pwm_m1 = motors[0].current_pwm;
  debug_pwm_m2 = motors[1].current_pwm;
  debug_pwm_m3 = motors[2].current_pwm;
  debug_pwm_m4 = motors[3].current_pwm;

  debug_delta_m1 = motors[0].encoder_delta;
  debug_delta_m2 = motors[1].encoder_delta;
  debug_delta_m3 = motors[2].encoder_delta;
  debug_delta_m4 = motors[3].encoder_delta;

  debug_total_m1 = motors[0].encoder_total_count;
  debug_total_m2 = motors[1].encoder_total_count;
  debug_total_m3 = motors[2].encoder_total_count;
  debug_total_m4 = motors[3].encoder_total_count;

  debug_speed_m1 = motors[0].speed_mps;
  debug_speed_m2 = motors[1].speed_mps;
  debug_speed_m3 = motors[2].speed_mps;
  debug_speed_m4 = motors[3].speed_mps;

  debug_target_m1 = motors[0].target_speed_mps;
  debug_target_m2 = motors[1].target_speed_mps;
  debug_target_m3 = motors[2].target_speed_mps;
  debug_target_m4 = motors[3].target_speed_mps;

  debug_error_m1 = motors[0].target_speed_mps - motors[0].speed_mps;
  debug_error_m2 = motors[1].target_speed_mps - motors[1].speed_mps;
  debug_error_m3 = motors[2].target_speed_mps - motors[2].speed_mps;
  debug_error_m4 = motors[3].target_speed_mps - motors[3].speed_mps;

  debug_pid_m1 = motors[0].pid.output;
  debug_pid_m2 = motors[1].pid.output;
  debug_pid_m3 = motors[2].pid.output;
  debug_pid_m4 = motors[3].pid.output;
}

/* =========================
   MOTION SPEED TEST TASK
   ========================= */

void Motion_Speed_TestTask(void)
{
  uint32_t now = HAL_GetTick();

  switch (motion_state)
  {
    case 0:
      /*
        State 1:
        4 bánh cùng tiến.
        Target mong muốn:
          M1 = +0.5
          M2 = +0.5
          M3 = +0.5
          M4 = +0.5
      */
      debug_test_state = 1;
      Motors_SetTargetSpeeds(0.5f, 0.5f, 0.5f, 0.5f);

      if (now - last_motion_time >= 5000)
      {
        last_motion_time = now;
        motion_state = 1;
      }
      break;

    case 1:
      /*
        State 2:
        4 bánh cùng lùi.
      */
      debug_test_state = 2;
      Motors_SetTargetSpeeds(-0.5f, -0.5f, -0.5f, -0.5f);

      if (now - last_motion_time >= 5000)
      {
        last_motion_time = now;
        motion_state = 2;
      }
      break;

    case 2:
      /*
        State 3:
        Test dấu chéo kiểu mecanum.
        Dùng để kiểm tra từng bánh có nhận target riêng hay chưa.
      */
      debug_test_state = 3;
      Motors_SetTargetSpeeds(0.5f, -0.5f, -0.5f, 0.5f);

      if (now - last_motion_time >= 5000)
      {
        last_motion_time = now;
        motion_state = 3;
      }
      break;

    case 3:
      /*
        State 4:
        Test dấu chéo ngược lại.
      */
      debug_test_state = 4;
      Motors_SetTargetSpeeds(-0.5f, 0.5f, 0.5f, -0.5f);

      if (now - last_motion_time >= 5000)
      {
        last_motion_time = now;
        motion_state = 4;
      }
      break;

    case 4:
      /*
        State 5:
        Test quay tại chỗ.
        Bên trái tiến, bên phải lùi.
      */
      debug_test_state = 5;
      Motors_SetTargetSpeeds(0.5f, -0.5f, 0.5f, -0.5f);

      if (now - last_motion_time >= 5000)
      {
        last_motion_time = now;
        motion_state = 5;
      }
      break;

    case 5:
      /*
        State 6:
        Dừng 3 giây rồi lặp lại.
      */
      debug_test_state = 6;
      Motors_SetTargetSpeeds(0.0f, 0.0f, 0.0f, 0.0f);

      if (now - last_motion_time >= 3000)
      {
        last_motion_time = now;
        motion_state = 0;
      }
      break;

    default:
      motion_state = 0;
      last_motion_time = now;
      break;
  }
}

/* =========================
   APP FUNCTIONS
   ========================= */

void App_Init(void)
{
  Motors_PWM_InitAll();
  Encoders_InitAll();

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    PID_Reset(&motors[i].pid);
    motors[i].target_speed_mps = 0.0f;
  }

  last_control_time = HAL_GetTick();
  last_motion_time = HAL_GetTick();
  motion_state = 0;

  Debug_UpdateVariables();
}

void App_Task(void)
{
  uint32_t now = HAL_GetTick();

  Motion_Speed_TestTask();

  if (now - last_control_time >= CONTROL_PERIOD_MS)
  {
    last_control_time = now;

    Motors_ControlUpdateAll();
    Debug_UpdateVariables();
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

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

  /*
    Encoder timers:
      TIM2 = Encoder Motor 1
      TIM3 = Encoder Motor 2
      TIM4 = Encoder Motor 3
      TIM5 = Encoder Motor 4
  */
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

  App_Init();

  while (1)
  {
    App_Task();
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

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}

#endif /* USE_FULL_ASSERT */
