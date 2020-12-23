/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 STMicroelectronics.
  * Copyright (c) 2020 fzawa5 (https://twitter.com/fzawa5).
  * All rights reserved.
  *
  * This software component is licensed by ST and fzawa5 under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <limits.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// amount of SPDIF sub-frame per block
#define SPDIF_SUBFRAMES_PER_BLOCK  (192 * 2)

// sub-frame transfer unit (multiple of SPDIF_SUBFRAMES_PER_BLOCK)
#define TRANSFER_UNIT  SPDIF_SUBFRAMES_PER_BLOCK

// debug macros
#ifdef DEBUG
#define debug_printf(fmt, ...)  printf((fmt), ## __VA_ARGS__)
#define debug_puts(str)      puts(str)
#else /* DEBUG */
#define debug_printf(fmt, ...)
#define debug_puts(str)
#endif /* DEBUG */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai1_a;

SPDIFRX_HandleTypeDef hspdif;
DMA_HandleTypeDef hdma_spdif_rx_dt;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// SPDIF & I2S RX context
struct {
  uint32_t buf[2][TRANSFER_UNIT];
  const uint32_t *readable_buf;
  size_t seq_num;
  bool active;
} spdif_rx = { {{0}}, NULL, 0, false }, i2s_rx = { {{0}}, NULL, 0, false };

// SPDIF TX context
struct {
  uint32_t buf[2][TRANSFER_UNIT];
  uint32_t *writable_buf;
  size_t seq_num;
} spdif_tx = { {{0}}, NULL, 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SAI1_Init(void);
static void MX_SPDIFRX_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * SPDIF RX: DMA half completed
 */
void HAL_SPDIFRX_RxHalfCpltCallback(SPDIFRX_HandleTypeDef *ctx)
{
  // buf[0] is ready for read
  spdif_rx.readable_buf = spdif_rx.buf[0];
  spdif_rx.seq_num++;
}

/*
 * SPDIF RX: DMA full completed
 */
void HAL_SPDIFRX_RxCpltCallback(SPDIFRX_HandleTypeDef *ctx)
{
  // buf[1] is ready for read
  spdif_rx.readable_buf = spdif_rx.buf[1];
  spdif_rx.seq_num++;
}

#ifdef DEBUG
void HAL_SPDIFRX_ErrorCallback(SPDIFRX_HandleTypeDef *ctx)
{
  printf("SPDIF RX: error occurred (%lu)\r\n", HAL_SPDIFRX_GetError(ctx));
}
#endif /* DEBUG */

/*
 * I2S RX: DMA half completed
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *ctx)
{
  // buf[0] is ready for read
  i2s_rx.readable_buf = i2s_rx.buf[0];
  i2s_rx.seq_num++;
}

/*
 * I2S RX: DMA full completed
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *ctx)
{
  // buf[1] is ready for read
  i2s_rx.readable_buf = i2s_rx.buf[1];
  i2s_rx.seq_num++;
}

/*
 * SPDIF TX: DMA half completed
 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *ctx)
{
  // buf[0] is ready for write
  spdif_tx.writable_buf = spdif_tx.buf[0];
  spdif_tx.seq_num++;
}

/*
 * SPDIF TX: DMA full completed
 */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *ctx)
{
  // buf[1] is ready for write
  spdif_tx.writable_buf = spdif_tx.buf[1];
  spdif_tx.seq_num++;
}

#ifdef DEBUG
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *ctx)
{
  if (ctx == &hsai_BlockA1) {
    printf("SPDIF TX: error occurred (%lu)\r\n", HAL_SAI_GetError(ctx));
  } else {
    printf("I2S RX: error occurred (%lu)\r\n", HAL_SAI_GetError(ctx));
  }
}
#endif /* DEBUG */

/*
 * check SPDIF & I2S RX activity periodically
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static size_t spdif_rx_prev_seq_num = 0, i2s_rx_prev_seq_num = 0;
  size_t __spdif_rx_seq_num, __i2s_rx_seq_num;

  // check SPDIF RX activity
  if (spdif_rx.active) {
    __spdif_rx_seq_num = spdif_rx.seq_num;
    if (__spdif_rx_seq_num == spdif_rx_prev_seq_num) {
      __HAL_SPDIFRX_CLEAR_IT(&hspdif, SPDIFRX_SR_SYNCD);
      spdif_rx.active = false;
    } else {
      spdif_rx_prev_seq_num = __spdif_rx_seq_num;
    }
  } else {
    if (__HAL_SPDIFRX_GET_FLAG(&hspdif, SPDIFRX_FLAG_SYNCD)) {
      spdif_rx.active = true;
    }
  }

  // check I2S RX activity
  __i2s_rx_seq_num = i2s_rx.seq_num;
  if (__i2s_rx_seq_num == i2s_rx_prev_seq_num) {
    i2s_rx.active = false;
  } else {
    i2s_rx_prev_seq_num = __i2s_rx_seq_num;
    i2s_rx.active = true;
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
  bool spdif_rx_prev_active = false, i2s_rx_prev_active = false;
  size_t spdif_tx_prev_seq_num = 0, spdif_rx_prev_seq_num = 0, i2s_rx_prev_seq_num = 0;
  size_t i;
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
  MX_USART2_UART_Init();
  MX_SAI1_Init();
  MX_SPDIFRX_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  puts("\r\nSPDIF & I2S Mixer Launched\r");
  debug_puts("debug mode enabled\r");

  // SPDIF RX: begin synchronization
  __HAL_SPDIFRX_SYNC(&hspdif);

  // I2S RX: begin DMA
  HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *) i2s_rx.buf, sizeof(i2s_rx.buf) / sizeof(i2s_rx.buf[0][0]));
  debug_puts("I2S RX: DMA started\r");

  // SPDIF TX: begin DMA
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *) spdif_tx.buf, sizeof(spdif_tx.buf) / sizeof(spdif_tx.buf[0][0]));
  debug_puts("SPDIF TX: DMA started\r");

  // enable interval timer (HAL_TIM_PeriodElapsedCallback() will be called periodically)
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    volatile bool __spdif_rx_active, __i2s_rx_active;
    volatile size_t __spdif_tx_seq_num;

    // SPDIF RX: begin DMA when sync is done / begin sync when signal is lost
    __spdif_rx_active = spdif_rx.active;
    if (__spdif_rx_active != spdif_rx_prev_active) {
      if (__spdif_rx_active) {
        debug_puts("SPDIF RX: sync done\r");
        HAL_SPDIFRX_ReceiveDataFlow_DMA(&hspdif, spdif_rx.buf[0], sizeof(spdif_rx.buf) / sizeof(spdif_rx.buf[0][0]));
        debug_puts("SPDIF RX: DMA started\r");
      } else {
        debug_puts("SPDIF RX: signal lost\r");
        HAL_DMA_Abort_IT(hspdif.hdmaDrRx);
        __HAL_SPDIFRX_IDLE(&hspdif);
        hspdif.State = HAL_SPDIFRX_STATE_READY;
        __HAL_SPDIFRX_SYNC(&hspdif);
        debug_puts("SPDIF RX: syncing\r");
      }
      spdif_rx_prev_active = __spdif_rx_active;
    }

    // I2S RX: restart DMA when signal is lost
    __i2s_rx_active = i2s_rx.active;
    if (__i2s_rx_active != i2s_rx_prev_active) {
      if (__i2s_rx_active) {
        debug_puts("I2S RX: signal detected\r");
      } else {
        debug_puts("I2S RX: signal lost\r");
        HAL_SAI_DMAStop(&hsai_BlockB1);
        HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *) i2s_rx.buf, sizeof(i2s_rx.buf) / sizeof(i2s_rx.buf[0][0]));
        debug_puts("I2S RX: DMA started\r");
      }
      i2s_rx_prev_active = __i2s_rx_active;
    }

    // SPDIF TX: write mixed audio samples (SPDIF RX & I2S RX)
    __spdif_tx_seq_num = spdif_tx.seq_num;
    if (__spdif_tx_seq_num != spdif_tx_prev_seq_num) {
      volatile size_t __spdif_rx_seq_num, __i2s_rx_seq_num;
      volatile const uint32_t *__spdif_rx_readable_buf = NULL, *__i2s_rx_readable_buf = NULL;
      volatile uint32_t *__spdif_tx_writable_buf = spdif_tx.writable_buf;

      if (__spdif_tx_seq_num - spdif_tx_prev_seq_num != 1) {
        debug_printf("SPDIF TX: skip %d sequences\r\n", __spdif_tx_seq_num - spdif_tx_prev_seq_num - 1);
      }

      // prepare SPDIF RX data
      __spdif_rx_seq_num = spdif_rx.seq_num;
      if (__spdif_rx_seq_num != spdif_rx_prev_seq_num) {
        if (__spdif_rx_seq_num - spdif_rx_prev_seq_num != 1) {
          debug_printf("I2S RX: skip %d sequences\r\n", __spdif_rx_seq_num - spdif_rx_prev_seq_num - 1);
        }
        __spdif_rx_readable_buf = spdif_rx.readable_buf;
        spdif_rx_prev_seq_num = __spdif_rx_seq_num;
      }

      // prepare I2S RX data
      __i2s_rx_seq_num = i2s_rx.seq_num;
      if (__i2s_rx_seq_num != i2s_rx_prev_seq_num) {
        if (__i2s_rx_seq_num - i2s_rx_prev_seq_num != 1) {
          debug_printf("I2S RX: skip %d sequences\r\n", __i2s_rx_seq_num - i2s_rx_prev_seq_num - 1);
        }
        __i2s_rx_readable_buf = i2s_rx.readable_buf;
        i2s_rx_prev_seq_num = __i2s_rx_seq_num;
      }

      // write SPDIF TX data
      for (i = 0; i < TRANSFER_UNIT; i++) {
        int32_t tx_data = 0;

        // mix SPDIF RX sample (24bit) only if available
        if (__spdif_rx_readable_buf) {
          tx_data = (int32_t) (__spdif_rx_readable_buf[i] << 8);
        }

        // mix I2S RX sample (32bit) only if available
        if (__i2s_rx_readable_buf) {
          int32_t rx_data = __i2s_rx_readable_buf[i];
          if (tx_data >= 0) {
              if (INT_MAX - tx_data <= rx_data) {
                rx_data = INT_MAX - tx_data;
              }
          } else {
              if (INT_MIN - tx_data >= rx_data) {
                rx_data = INT_MIN - tx_data;
              }
          }
          tx_data += rx_data;
        }

        // write mixed sample (24bit)
        __spdif_tx_writable_buf[i] = ((uint32_t) tx_data) >> 8;
      }

      // turn on LED during sending audio samples
      if (__spdif_rx_readable_buf || __i2s_rx_readable_buf) {
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      } else {
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }

      spdif_tx_prev_seq_num = __spdif_tx_seq_num;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SPDIFRX;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 5;
  PeriphClkInitStruct.SpdifClockSelection = RCC_SPDIFRXCLKSOURCE_PLLR;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_SPDIF_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_FULL;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_96K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SPDIFRX Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPDIFRX_Init(void)
{

  /* USER CODE BEGIN SPDIFRX_Init 0 */

  /* USER CODE END SPDIFRX_Init 0 */

  /* USER CODE BEGIN SPDIFRX_Init 1 */

  /* USER CODE END SPDIFRX_Init 1 */
  hspdif.Instance = SPDIFRX;
  hspdif.Init.InputSelection = SPDIFRX_INPUT_IN0;
  hspdif.Init.Retries = SPDIFRX_MAXRETRIES_NONE;
  hspdif.Init.WaitForActivity = SPDIFRX_WAITFORACTIVITY_ON;
  hspdif.Init.ChannelSelection = SPDIFRX_CHANNEL_A;
  hspdif.Init.DataFormat = SPDIFRX_DATAFORMAT_LSB;
  hspdif.Init.StereoMode = SPDIFRX_STEREOMODE_ENABLE;
  hspdif.Init.PreambleTypeMask = SPDIFRX_PREAMBLETYPEMASK_OFF;
  hspdif.Init.ChannelStatusMask = SPDIFRX_CHANNELSTATUS_OFF;
  hspdif.Init.ValidityBitMask = SPDIFRX_VALIDITYMASK_OFF;
  hspdif.Init.ParityErrorMask = SPDIFRX_PARITYERRORMASK_OFF;
  if (HAL_SPDIFRX_Init(&hspdif) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPDIFRX_Init 2 */

  /* USER CODE END SPDIFRX_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
