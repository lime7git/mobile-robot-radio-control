/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_NRF24.h"

#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "robot.h"
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

sEncoder encoder_left;
sEncoder encoder_right;

sPid pid_left;
sPid pid_right;

sMotor motor_left;
sMotor motor_right;

sRadioFrame radio_frame;

sRobot Robot;

uint32_t led_tick = 0;
uint32_t received_packet = 0;


//nRF24 
uint64_t rx_pipe_address = 0x1111111111;
uint32_t ack_payload = 123456;
uint16_t joystick[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_UPDATE(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

	PID_INIT(&pid_left, 	0.1f, 0.03f, 0.001f, 3500);
	PID_INIT(&pid_right, 	0.095f, 0.021f, 0.001f, 3500);
	
	ENCODER_INIT(&encoder_left, 	&htim3);
	ENCODER_INIT(&encoder_right, 	&htim4);
	
	MOTOR_INIT(&motor_left, 	&htim1, TIM_CHANNEL_1, MOTL_IN4_GPIO_Port, MOTL_IN4_Pin, MOTL_IN3_GPIO_Port, MOTL_IN3_Pin, &encoder_left,  &pid_left);
	MOTOR_INIT(&motor_right, 	&htim2, TIM_CHANNEL_2, MOTR_IN1_GPIO_Port, MOTR_IN1_Pin, MOTR_IN2_GPIO_Port, MOTR_IN2_Pin, &encoder_right, &pid_right);
	
	ROBOT_INIT(&Robot, &motor_left, &motor_right, &radio_frame);

	HAL_TIM_Encoder_Start(Robot.motor_left->encoder->encoder_timer,  TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(Robot.motor_right->encoder->encoder_timer, TIM_CHANNEL_ALL);
	
	HAL_TIM_PWM_Start(Robot.motor_left->pwm_timer,  Robot.motor_left->pwm_timer_channel);
	HAL_TIM_PWM_Start(Robot.motor_right->pwm_timer, Robot.motor_right->pwm_timer_channel);
	
	HAL_TIM_Base_Start_IT(&htim5);

	NRF24_begin(GPIOA, SPI3_CS_Pin, nrf_CE_PIN, hspi3);
	
	NRF24_setAutoAck(true);
	NRF24_setChannel(50);
	NRF24_setPayloadSize(6);
	NRF24_openReadingPipe(1, rx_pipe_address);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();
	
	NRF24_startListening();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if(NRF24_available())
		{
			if(NRF24_read(Robot.radio_frame, 6)) 
			{
				received_packet++;
				NRF24_writeAckPayload(1, &ack_payload, 6);
			}
		}
		
		LED_UPDATE();
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

/* USER CODE BEGIN 4 */
void LED_UPDATE(void)
{
	
	if(HAL_GetTick() - led_tick >= 250)
	{
		HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
		led_tick = HAL_GetTick();
	}
	
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)				// ################## przerwanie od TIMera aby uzyskac przerwanie co 1ms  ######################################################
{
  /* Prevent unused argument(s) compilation warning */
 //UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */

	MOTOR_UPDATE_VELOCITY(Robot.motor_left);
	MOTOR_UPDATE_VELOCITY(Robot.motor_right);
	
	ROBOT_UPDATE_VELOCITY(&Robot);
	
	MOTOR_UPDATE_PID(Robot.motor_left);
	MOTOR_UPDATE_PID(Robot.motor_right);
	
	PID_CALCULATE(Robot.motor_left->pid);
	PID_CALCULATE(Robot.motor_right->pid);
	
	if(Robot.radio_frame->enable == 1) 
	{
		MOTOR_UPDATE_PWM(Robot.motor_left);
		MOTOR_UPDATE_PWM(Robot.motor_right);
	}
	else
	{
		MOTOR_STOP(Robot.motor_left);
		MOTOR_STOP(Robot.motor_right);
		PID_RESET(Robot.motor_left->pid);
		PID_RESET(Robot.motor_right->pid);
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
