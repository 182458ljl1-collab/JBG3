/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include <stdio.h>
#include <stdint.h>
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

/* USER CODE BEGIN PV */
#define PWM_MAX        3599.0f
#define PID_DT         0.02f
#define ENCODER_CPR    17600.0f
#define ENCODER_DIR      1.0f
#define TARGET_RPM_MAX 120.0f
#define VOFA_RPM_SCALE  100.0f
#define VOFA_ERR_SCALE  100.0f
#define RPM_FILTER_ALPHA 0.25f
#define PWM_STEP_MAX     80.0f
#define SPEED_FF_OFFSET 330.0f
#define SPEED_FF_GAIN   158.0f
#define PID_CORRECTION_MAX 600.0f

PID_t motor_pid;

volatile float target_rpm = 5.0f;
volatile float actual_rpm = 0.0f;
volatile float raw_rpm = 0.0f;
volatile float motor_pwm = 0.0f;
volatile float pid_pwm = 0.0f;
volatile int16_t encoder_count = 0;

volatile uint8_t vofa_send_flag = 0;
volatile uint16_t vofa_count = 0;
char vofa_tx_buf[160];
#define UART_CMD_BUF_LEN  16
volatile uint8_t uart_rx_data = 0;
volatile uint8_t uart_rx_index = 0;
volatile uint8_t uart_cmd_ready = 0;
char uart_rx_buf[UART_CMD_BUF_LEN];
char uart_cmd_buf[UART_CMD_BUF_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float ClampFloat(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

static float Speed_FeedForward(float rpm)
{
    float abs_rpm;
    float pwm;

    if (rpm > -0.01f && rpm < 0.01f)
    {
        return 0.0f;
    }

    abs_rpm = rpm > 0.0f ? rpm : -rpm;
    pwm = SPEED_FF_OFFSET + SPEED_FF_GAIN * abs_rpm;

    if (rpm < 0.0f)
    {
        pwm = -pwm;
    }

    return ClampFloat(pwm, -PWM_MAX, PWM_MAX);
}

void Motor_SetPWM(float pwm)
{
    uint16_t duty = 0;

    if (pwm > PWM_MAX)
        pwm = PWM_MAX;

    if (pwm < -PWM_MAX)
        pwm = -PWM_MAX;

    if (pwm > 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

        duty = (uint16_t)pwm;
    }
    else if (pwm < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

        duty = (uint16_t)(-pwm);
    }
    else
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

        duty = 0;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
}

void Motor_Stop(void)
{
    PID_Reset(&motor_pid);
    Motor_SetPWM(0);
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

float UART_ParseFloat(const char *str)
{
    float value = 0.0f;
    float sign = 1.0f;
    float decimal = 0.1f;
    uint8_t after_dot = 0;

    if (*str == '-')
    {
        sign = -1.0f;
        str++;
    }
    else if (*str == '+')
    {
        str++;
    }

    while (*str != '\0')
    {
        if (*str == '.')
        {
            after_dot = 1;
        }
        else if (*str >= '0' && *str <= '9')
        {
            if (after_dot)
            {
                value += (float)(*str - '0') * decimal;
                decimal *= 0.1f;
            }
            else
            {
                value = value * 10.0f + (float)(*str - '0');
            }
        }
        else
        {
            break;
        }

        str++;
    }

    return value * sign;
}

void UART_ProcessCommand(void)
{
    if (uart_cmd_ready)
    {
        uart_cmd_ready = 0;
        target_rpm = ClampFloat(UART_ParseFloat(uart_cmd_buf),
                                -TARGET_RPM_MAX,
                                TARGET_RPM_MAX);
    }
}

void UART1_SendString(const char *str)
{
    while (*str != '\0')
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)str, 1, 10);
        str++;
    }
}

void VOFA_SendData(void)
{
    int len = snprintf(vofa_tx_buf, sizeof(vofa_tx_buf),
                       "%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f\r\n",
                       target_rpm,
                       actual_rpm,
                       raw_rpm,
                       actual_rpm * VOFA_RPM_SCALE,
                       motor_pid.error,
                       encoder_count,
                       motor_pwm,
                       motor_pid.i_out);

    if (len > 0)
    {
        if (len > (int)sizeof(vofa_tx_buf))
        {
            len = sizeof(vofa_tx_buf);
        }

        HAL_UART_Transmit(&huart1, (uint8_t *)vofa_tx_buf, (uint16_t)len, 100);
    }
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    HAL_TIM_Base_Start_IT(&htim4);

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    PID_Init(&motor_pid, 25.0f, 5.0f, 0.0f, -PID_CORRECTION_MAX, PID_CORRECTION_MAX);

    Motor_SetPWM(0);
    UART1_SendString("uart1_vofa_start\r\n");
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        UART_ProcessCommand();
        if (vofa_send_flag)
        {
            vofa_send_flag = 0;
            VOFA_SendData();
        }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);

        raw_rpm = ((float)encoder_count * ENCODER_DIR * 60.0f) / (ENCODER_CPR * PID_DT);
        actual_rpm += (raw_rpm - actual_rpm) * RPM_FILTER_ALPHA;

        if (target_rpm > -0.01f && target_rpm < 0.01f)
        {
            Motor_Stop();
            motor_pwm = 0.0f;
            pid_pwm = 0.0f;
        }
        else
        {
            pid_pwm = Speed_FeedForward(target_rpm) + PID_Update(&motor_pid, target_rpm, actual_rpm, PID_DT);
            pid_pwm = ClampFloat(pid_pwm, -PWM_MAX, PWM_MAX);
            if (pid_pwm > motor_pwm + PWM_STEP_MAX)
            {
                motor_pwm += PWM_STEP_MAX;
            }
            else if (pid_pwm < motor_pwm - PWM_STEP_MAX)
            {
                motor_pwm -= PWM_STEP_MAX;
            }
            else
            {
                motor_pwm = pid_pwm;
            }
            Motor_SetPWM(motor_pwm);
        }

        /*
         * TIM4 �� 20ms һ�Ρ�
         * vofa_count >= 5 ��ʾ 100ms ��һ�����ݣ�Ҳ���� 10Hz��
         */
        vofa_count++;
        if (vofa_count >= 5)
        {
            vofa_count = 0;
            vofa_send_flag = 1;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;

    if (huart->Instance == USART1)
    {
        if (uart_rx_data == '\r' || uart_rx_data == '\n')
        {
            if (uart_rx_index > 0)
            {
                for (i = 0; i < uart_rx_index; i++)
                {
                    uart_cmd_buf[i] = uart_rx_buf[i];
                }
                uart_cmd_buf[uart_rx_index] = '\0';
                uart_rx_index = 0;
                uart_cmd_ready = 1;
            }
        }
        else if ((uart_rx_data >= '0' && uart_rx_data <= '9') ||
                 uart_rx_data == '.' || uart_rx_data == '-' || uart_rx_data == '+')
        {
            if (uart_rx_index < UART_CMD_BUF_LEN - 1)
            {
                uart_rx_buf[uart_rx_index++] = uart_rx_data;
            }
            else
            {
                uart_rx_index = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_data, 1);
    }
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
