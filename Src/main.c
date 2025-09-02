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
#include "xensiv_bgt60trxx.h"
#include "xensiv_bgt60trxx_platform.h"
#include "xensiv_bgt60trxx_conf.h"
#include "arm_math.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM_MATH_MVEF
#define M_PI 3.14159265358979323846
#define N_SAMPLES XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
bool firstdata = false;
xensiv_bgt60trxx_t dev;
static void hann_init(void);
static void blackman_init(void);
static inline void apply_window(float *x /* len = 1024 */);
static inline void fftmag(float32_t * inp,float32_t * mag,int len);
static inline void avgmag(float32_t * mag,int len, int div);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t sum = 0;
uint16_t avg = 0;
uint16_t data[N_SAMPLES] = {};
float32_t data2[N_SAMPLES] = {};
uint32_t fftSize = N_SAMPLES;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
arm_rfft_fast_instance_f32 rfft;
arm_status status;
uint32_t maxindex = 0;
float32_t freqbin[N_SAMPLES];
float32_t rangebin[N_SAMPLES];
float32_t maxValue;
float32_t distsum = 0;
float32_t distance = 0;
static float win[N_SAMPLES];     // coefficients
static const float hann_gain = 0.5f;
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
  MX_ICACHE_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(en_ldo_radar_GPIO_Port,en_ldo_radar_Pin,1);
  HAL_GPIO_WritePin(osc_en_GPIO_Port,osc_en_Pin,1);
  HAL_GPIO_WritePin(Translator_OE_GPIO_Port,Translator_OE_Pin,1);
  HAL_GPIO_WritePin(led_select0_GPIO_Port,led_select0_Pin,0);
  HAL_GPIO_WritePin(led_select1_GPIO_Port,led_select1_Pin,0);
  HAL_Delay(100);

  dev.iface = &hspi1;
  xensiv_bgt60trxx_hard_reset(&dev);
  int32_t check1 = xensiv_bgt60trxx_init(&dev, &hspi1,  false);
  int32_t check0 = xensiv_bgt60trxx_config(&dev,register_list,40);
  

  for(size_t i = 0; i < 1024;++i){
    freqbin[i] = i*(XENSIV_BGT60TRXX_CONF_SAMPLE_RATE/(N_SAMPLES));
    rangebin[i] = ((299792458.0f)*XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S*(i*(XENSIV_BGT60TRXX_CONF_SAMPLE_RATE/(N_SAMPLES))))/((float32_t)2*(XENSIV_BGT60TRXX_CONF_END_FREQ_HZ - XENSIV_BGT60TRXX_CONF_START_FREQ_HZ));
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  status=arm_rfft_fast_init_f32(&rfft, 1024);            
  blackman_init(); //hann_init();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    for(int i =0; i < 5; ++i){
      float32_t mag[512] = {};    
      uint16_t data[1024] = {};
      float32_t data2[1024] = {};
      float32_t fftoutput[1024] = {};       
      uint32_t check2 = xensiv_bgt60trxx_soft_reset(&dev,XENSIV_BGT60TRXX_RESET_FIFO);
      uint32_t check3 = xensiv_bgt60trxx_start_frame(&dev,true);
      while(!(HAL_GPIO_ReadPin(IRQ_R_M_GPIO_Port,IRQ_R_M_Pin))){}
      xensiv_bgt60trxx_get_fifo_data(&dev,data,1024);

      sum = 0;
      for(size_t i = 0; i < N_SAMPLES; ++i){ //Remove DC bias
        sum += (float) data[i];
      }
      avg = sum/N_SAMPLES;
      for(size_t i = 0; i < N_SAMPLES; ++i){
        data2[i] = (float)(data[i]) - avg;

      }  
      apply_window(data2);

      arm_rfft_fast_f32(&rfft, data2, fftoutput, ifftFlag);

      fftmag(fftoutput,mag,N_SAMPLES/2);
          status = ARM_MATH_SUCCESS;
      for(int i = 0; i < 10; ++i){
        mag[i] = 0;
      }

      arm_max_f32(mag, N_SAMPLES/2, &maxValue, &maxindex); 
      distsum += rangebin[maxindex];
    }
  
  distance = distsum/5; 
  if(distance < 1.5){
    HAL_GPIO_WritePin(led_select1_GPIO_Port,led_select1_Pin,0);
    HAL_GPIO_WritePin(led_select0_GPIO_Port,led_select0_Pin,1);
    HAL_Delay(100);
  }
  else{
    HAL_GPIO_WritePin(led_select1_GPIO_Port,led_select1_Pin,1);
    HAL_GPIO_WritePin(led_select0_GPIO_Port,led_select0_Pin,0);
    HAL_Delay(100);
  }
  distsum = 0;
}    
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  *  Warning : Only applied when the LSE is disabled.
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 10;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the CRS APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin|led_select0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Translator_OE_Pin|en_ldo_radar_Pin|osc_en_Pin|RST_M_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Translator_OE_Pin en_ldo_radar_Pin osc_en_Pin RST_M_R_Pin */
  GPIO_InitStruct.Pin = Translator_OE_Pin|en_ldo_radar_Pin|osc_en_Pin|RST_M_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : led_select0_Pin */
  GPIO_InitStruct.Pin = led_select0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_select0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led_select1_Pin */
  GPIO_InitStruct.Pin = led_select1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(led_select1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_R_M_Pin */
  GPIO_InitStruct.Pin = IRQ_R_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_R_M_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_SPI_StateTypeDef SPI_ChangeDataSize(SPI_HandleTypeDef * spidev, unsigned long datasizemacro){
    
  HAL_SPI_StateTypeDef spistatus = HAL_SPI_STATE_BUSY;
  while(spistatus != 1){
    spistatus = HAL_SPI_GetState(spidev);
  }
  HAL_SPIEx_FlushRxFifo(spidev);
  bool transferdone = false;
  while(!transferdone){
    transferdone = __HAL_SPI_GET_FLAG(spidev,SPI_FLAG_TXC);
  }
  __HAL_SPI_DISABLE(spidev);
  spidev->Init.DataSize = datasizemacro;
  MODIFY_REG(spidev->Instance->CFG1, SPI_CFG1_DSIZE, datasizemacro);
  __HAL_SPI_ENABLE(spidev);

  return HAL_SPI_GetState(spidev);
}
static void hann_init(void)
{
    const float k = 2.0f * (float)M_PI / (float)(N_SAMPLES - 1);
    for (uint32_t n = 0; n < N_SAMPLES; ++n) {
        win[n] = 0.5f * (1.0f - cosf(k * (float)n));
    }
}

static inline void apply_window(float *x /* len = 1024 */)
{
    for (uint32_t n = 0; n < N_SAMPLES; ++n) {
        x[n] *= win[n];
    }
}

static inline void fftmag(float32_t * inp,float32_t * mag,int len){
  float32_t re;
  float32_t im;
  mag[0] = 0.0f;
  mag[len-1] = fabsf(inp[1]);
  for(int i =1; i < (len-1); ++i){
    re = inp[2U*i + 0U];
    im = inp[2U*i + 1U];
    mag[i] = sqrtf(re*re + im*im);
  }
}

static inline void avgmag(float32_t * mag,int len, int div){
  for(int i = 0; i < len; ++i){
    mag[i] = mag[i]/div;
  }
}

static void blackman_init(void){
  float a0 = (float) 7938/18608;
  float a1 = (float) 9240/18608;
  float a2 = (float) 1430/18608;
  for(uint32_t n = 0; n < N_SAMPLES; ++n){
    win[n] = a0 - a1*cosf((float)(2*PI*n)/N_SAMPLES) +a2*cosf((float)(4*PI*n)/N_SAMPLES);
  }
}

static inline void cacfar(float32_t * fft, float32_t Pfa, int guard, int training){
  float32_t alpha = N_SAMPLES*(powf(Pfa,(float)-1/N_SAMPLES) - 1);
  float32_t threshold;
  float32_t suml = 0;
  float32_t sumr = 0;
  for(int i = (guard + training - 1); i < (N_SAMPLES - (guard + training)); ++i){
    for(int j = i, k = i; j < (i + guard + training), k > (i - (guard + training - 1)); ++j, --k){
      suml += fft[k];
      sumr += fft[j];
    }
    threshold = sum
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
