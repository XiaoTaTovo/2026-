/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

#include "bsp_bluetooth.h"
#include "bsp_uart_dma.h"
#include "ball_observation_protocol.h"
#include "chassis_feedforward_protocol.h"
#include "pitch_axis_manual_control.h"
#include "pitch_axis_manual_telemetry.h"
#include "pitch_axis_self_test.h"
#include "pitch_axis_self_test_telemetry.h"
#include "pitch_axis_vision_control.h"
#include "pitch_axis_vision_telemetry.h"
#include "x42s_driver.h"

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

static BspBluetooth bluetooth_port;
static volatile BspBluetoothResult bluetooth_init_result;
static volatile BspBluetoothResult bluetooth_start_result;
static BspUartDmaPort vision_port;
static BspUartDmaResult vision_init_result;
static BspUartDmaResult vision_start_result;
static BallObservationParser vision_parser;
static PitchAxisVisionControl pitch_vision_control;
static PitchAxisVisionTelemetry pitch_vision_telemetry;
static bool pitch_vision_initialized;
static BspUartDmaPort chassis_port;
static BspUartDmaResult chassis_init_result;
static BspUartDmaResult chassis_start_result;
static ChassisFeedforwardParser chassis_parser;
static ChassisFeedforwardSample chassis_sample;
static ChassisFeedforwardEstimate chassis_estimate;
static uint32_t chassis_last_telemetry_ms;
static X42sDriver pitch_x42;
static X42sDriverResult pitch_x42_init_result;
static X42sDriverResult pitch_x42_start_result = X42S_DRIVER_NOT_STARTED;
static PitchAxisSelfTest pitch_self_test;
static PitchAxisSelfTestResult pitch_self_test_init_result =
    PITCH_AXIS_SELF_TEST_NOT_READY;
static PitchAxisSelfTestResult pitch_self_test_start_result =
    PITCH_AXIS_SELF_TEST_NOT_READY;
static PitchAxisSelfTestTelemetry pitch_self_test_telemetry;
static PitchAxisManualControl pitch_manual_control;
static PitchAxisManualTelemetry pitch_manual_telemetry;
static bool pitch_manual_initialized;
static const PitchAxisManualConfig pitch_manual_config = {
    .address = X42S_DEFAULT_ADDRESS,
    .positive_direction = 0U,
    .negative_direction = 1U,
    .speed_rpm = 60U,
    .acceleration = 100U,
    .step_pulses = 32U,
    /* Mode 0 accumulates each command from the previous input target.  The
     * installed motor firmware treated mode 2 as an absolute target during
     * the first hardware trial, so manual button steps use the vendor
     * example's repeatable relative mode. */
    .motion_mode = 0U,
    .synchronize = false,
    .debounce_ms = 30U,
    .settle_ms = 1200U
};
static const PitchAxisVisionConfig pitch_vision_config = {
    .target_position_0_1mm = 0,
    .deadband_0_1mm = 5,
    /* The camera must mark a detection valid, then this second gate rejects
     * weak detections before they can become an EMM command candidate. */
    .minimum_confidence_permille = 700U,
    .maximum_observation_age_ms = 150U,
    .control_period_ms = 50U,
    /* MEASURED (single rough trial): 30 * 32 pulses produced about 1 mm.
     * This remains a configurable estimate until repeated ruler calibration. */
    .pulses_per_mm = 960U,
    .minimum_command_pulses = 8U,
    /* UNVERIFIED initial gains. The first firmware is dry-run only and will
     * log candidates rather than transmit them to the motor. */
    .kp_mm_per_mm = 0.10f,
    .ki_per_s = 0.0f,
    .kd_s = 0.0f,
    .integral_limit_mm_s = 10.0f,
    .output_limit_mm = 0.20f,
    .positive_error_uses_positive_direction = true
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void ChassisFeedforward_Report(uint32_t now_ms)
{
  char text[192];
  uint32_t age_ms;

  if ((uint32_t)(now_ms - chassis_last_telemetry_ms) < 500U)
  {
    return;
  }
  chassis_last_telemetry_ms = now_ms;
  age_ms = chassis_sample.valid ? (uint32_t)(now_ms - chassis_sample.received_ms) :
                                  0xFFFFFFFFU;
  (void)snprintf(
      text,
      sizeof(text),
      "CHASSIS_FF,state=LOCKED,frames=%lu,seq=%u,age=%lu,cmd=%d,imu=%d,est=%d,tilt_mrad=%d,crc=%lu,fmt=%lu\r\n",
      (unsigned long)chassis_parser.accepted_frame_count,
      (unsigned int)chassis_sample.sequence,
      (unsigned long)age_ms,
      (int)chassis_sample.command_accel_mm_s2,
      (int)chassis_sample.imu_accel_mm_s2,
      (int)chassis_estimate.estimated_accel_mm_s2,
      (int)chassis_estimate.pitch_candidate_mrad,
      (unsigned long)chassis_parser.crc_error_count,
      (unsigned long)chassis_parser.format_error_count);
  (void)BspBluetooth_WriteString(&bluetooth_port, text);
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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  bluetooth_init_result = BspBluetooth_Init(&bluetooth_port, &huart1);
  if (bluetooth_init_result == BSP_BLUETOOTH_OK)
  {
    bluetooth_start_result = BspBluetooth_Start(&bluetooth_port);
    if (bluetooth_start_result == BSP_BLUETOOTH_OK)
    {
      (void)BspBluetooth_WriteString(&bluetooth_port, "BT_DMA_READY\r\n");
    }
  }

  vision_init_result = BspUartDma_Init(&vision_port, &huart6);
  if (vision_init_result == BSP_UART_DMA_OK)
  {
    vision_start_result = BspUartDma_Start(&vision_port);
  }
  BallObservationParser_Init(&vision_parser);
  pitch_vision_initialized = PitchAxisVisionControl_Init(
      &pitch_vision_control,
      &pitch_vision_config,
      HAL_GetTick());
  if ((vision_init_result == BSP_UART_DMA_OK) &&
      (vision_start_result == BSP_UART_DMA_OK) &&
      pitch_vision_initialized)
  {
    (void)PitchAxisVisionTelemetry_Init(
        &pitch_vision_telemetry,
        &bluetooth_port,
        &vision_port,
        &vision_parser,
        &pitch_vision_control,
        HAL_GetTick());
    (void)BspBluetooth_WriteString(&bluetooth_port, "VISION_UART_DMA_READY\r\n");
  }
  else
  {
    (void)BspBluetooth_WriteString(&bluetooth_port, "VISION_INIT_ERROR\r\n");
  }

  chassis_init_result = BspUartDma_Init(&chassis_port, &huart2);
  if (chassis_init_result == BSP_UART_DMA_OK)
  {
    chassis_start_result = BspUartDma_Start(&chassis_port);
  }
  ChassisFeedforwardParser_Init(&chassis_parser);
  if ((chassis_init_result == BSP_UART_DMA_OK) &&
      (chassis_start_result == BSP_UART_DMA_OK))
  {
    (void)BspBluetooth_WriteString(&bluetooth_port,
                                   "CHASSIS_UART_DMA_READY\r\n");
  }
  else
  {
    (void)BspBluetooth_WriteString(&bluetooth_port,
                                   "CHASSIS_UART_DMA_INIT_ERROR\r\n");
  }

  pitch_x42_init_result = X42sDriver_Init(&pitch_x42, &huart3);
  if (pitch_x42_init_result == X42S_DRIVER_OK)
  {
    pitch_x42_start_result = X42sDriver_Start(&pitch_x42);
  }

  pitch_manual_initialized = PitchAxisManualControl_Init(
      &pitch_manual_control,
      &pitch_x42,
      &pitch_manual_config,
      HAL_GetTick());
  if (pitch_manual_initialized)
  {
    (void)PitchAxisManualTelemetry_Init(
        &pitch_manual_telemetry,
        &pitch_manual_control,
        &bluetooth_port,
        HAL_GetTick());
  }

  if (pitch_x42_start_result == X42S_DRIVER_OK)
  {
    pitch_self_test_init_result = PitchAxisSelfTest_Init(
        &pitch_self_test,
        &pitch_x42,
        X42S_DEFAULT_ADDRESS,
        PITCH_AXIS_SELF_TEST_DEFAULT_CYCLES,
        PITCH_AXIS_SELF_TEST_DEFAULT_PERIOD_MS,
        PITCH_AXIS_SELF_TEST_DEFAULT_START_DELAY_MS);
    if (pitch_self_test_init_result == PITCH_AXIS_SELF_TEST_OK)
    {
      pitch_self_test_start_result = PitchAxisSelfTest_Start(
          &pitch_self_test,
          HAL_GetTick());
    }
  }

  if ((pitch_self_test_start_result != PITCH_AXIS_SELF_TEST_OK) ||
      !PitchAxisSelfTestTelemetry_Init(
          &pitch_self_test_telemetry,
          &pitch_self_test,
          &bluetooth_port))
  {
    (void)BspBluetooth_WriteString(
        &bluetooth_port,
        "PITCH_READ1000_INIT_ERROR\r\n");
  }

  if (!pitch_manual_initialized)
  {
    (void)BspBluetooth_WriteString(
        &bluetooth_port,
        "PITCH_MANUAL_INIT_ERROR\r\n");
  }
  else if (pitch_self_test_start_result != PITCH_AXIS_SELF_TEST_OK)
  {
    PitchAxisManualControl_SetCommunicationResult(
        &pitch_manual_control,
        false,
        HAL_GetTick());
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint8_t rx_data[64];
    uint8_t vision_rx_data[64];
    uint8_t chassis_rx_data[64];
    size_t available;
    size_t read_length;
    size_t transfer_length;
    size_t vision_read_length;
    size_t vision_index;
    size_t chassis_read_length;
    size_t chassis_index;
    uint32_t now_ms;
    PitchAxisManualButtons buttons;
    PitchAxisSelfTestState self_test_state;
    BallObservation observation;

    now_ms = HAL_GetTick();
    BspBluetooth_Service(&bluetooth_port);
    BspUartDma_Service(&vision_port);
    BspUartDma_Service(&chassis_port);
    buttons.key1_pressed =
        (HAL_GPIO_ReadPin(Key_1_GPIO_Port, Key_1_Pin) == GPIO_PIN_RESET);
    buttons.key2_pressed =
        (HAL_GPIO_ReadPin(Key_2_GPIO_Port, Key_2_Pin) == GPIO_PIN_RESET);
    buttons.key3_pressed =
        (HAL_GPIO_ReadPin(Key_3_GPIO_Port, Key_3_Pin) == GPIO_PIN_RESET);
    buttons.key4_pressed =
        (HAL_GPIO_ReadPin(Key_4_GPIO_Port, Key_4_Pin) == GPIO_PIN_RESET);

    if (pitch_manual_initialized)
    {
      PitchAxisManualControl_Service(
          &pitch_manual_control,
          now_ms,
          buttons);
    }

    do
    {
      vision_read_length = BspUartDma_Read(
          &vision_port,
          vision_rx_data,
          sizeof(vision_rx_data));
      for (vision_index = 0U; vision_index < vision_read_length; ++vision_index)
      {
        if (BallObservationParser_OnByte(
                &vision_parser,
                vision_rx_data[vision_index],
                now_ms,
                &observation))
        {
          PitchAxisVisionControl_OnObservation(
              &pitch_vision_control,
              &observation);
        }
      }
    } while (vision_read_length == sizeof(vision_rx_data));
    do
    {
      chassis_read_length = BspUartDma_Read(
          &chassis_port,
          chassis_rx_data,
          sizeof(chassis_rx_data));
      for (chassis_index = 0U;
           chassis_index < chassis_read_length;
           ++chassis_index)
      {
        ChassisFeedforwardSample decoded_sample;

        if (ChassisFeedforwardParser_OnByte(
                &chassis_parser,
                chassis_rx_data[chassis_index],
                now_ms,
                &decoded_sample))
        {
          chassis_sample = decoded_sample;
          ChassisFeedforwardEstimate_Update(
              &chassis_estimate,
              &chassis_sample);
        }
      }
    } while (chassis_read_length == sizeof(chassis_rx_data));
    ChassisFeedforward_Report(now_ms);
    if (pitch_vision_initialized)
    {
      PitchAxisVisionControl_Service(&pitch_vision_control, now_ms);
      PitchAxisVisionTelemetry_Service(&pitch_vision_telemetry, now_ms);
    }

    if (!pitch_manual_initialized ||
        ((PitchAxisManualControl_GetState(&pitch_manual_control) !=
          PITCH_MANUAL_STATE_STOPPING) &&
         (PitchAxisManualControl_GetState(&pitch_manual_control) !=
          PITCH_MANUAL_STATE_FAULT_LATCHED)))
    {
      PitchAxisSelfTest_Service(&pitch_self_test, now_ms);
    }

    self_test_state = PitchAxisSelfTest_GetState(&pitch_self_test);
    if (pitch_manual_initialized &&
        (self_test_state == PITCH_AXIS_SELF_TEST_STATE_COMM_PASS))
    {
      PitchAxisManualControl_SetCommunicationResult(
          &pitch_manual_control,
          true,
          now_ms);
    }
    else if (pitch_manual_initialized &&
             (self_test_state == PITCH_AXIS_SELF_TEST_STATE_FAILED))
    {
      PitchAxisManualControl_SetCommunicationResult(
          &pitch_manual_control,
          false,
          now_ms);
    }

    PitchAxisSelfTestTelemetry_Service(&pitch_self_test_telemetry);
    if (pitch_manual_initialized)
    {
      PitchAxisManualTelemetry_Service(&pitch_manual_telemetry, now_ms);
    }

    available = BspBluetooth_Available(&bluetooth_port);
    transfer_length = available;
    if (transfer_length > sizeof(rx_data))
    {
      transfer_length = sizeof(rx_data);
    }

    if ((transfer_length > 0U) &&
        (BspBluetooth_Read(
             &bluetooth_port,
             rx_data,
             transfer_length,
             &read_length) == BSP_BLUETOOTH_OK))
    {
      (void)read_length;
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
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
