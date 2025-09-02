/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
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
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "xensiv_bgt60trxx.h"
#include "xensiv_bgt60trxx_platform.h"
#include "xensiv_bgt60trxx_conf.h"
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
/* USER CODE BEGIN Variables */
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

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
float32_t thres[512];
static float win[N_SAMPLES];     // coefficients
static const float hann_gain = 0.5f;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
xensiv_bgt60trxx_t dev;
static void hann_init(void);
static void blackman_init(void);
static inline void apply_window(float *x /* len = 1024 */);
static inline void fftmag(float32_t * inp,float32_t * mag,int len);
static inline void avgmag(float32_t * mag,int len, int div);
static inline void cacfar(float32_t * fftmag, float32_t * threshold, float32_t Pfa, int guard, int training);
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
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
  status=arm_rfft_fast_init_f32(&rfft, 1024);            
  blackman_init(); //hann_init();
  /* Infinite loop */
  for(;;)
  {
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

      memset(mag,0,10*sizeof(float32_t));
      cacfar(mag,thres,0.05,3,7);
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
 
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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
  /*
  X = { real[0], imag[0], real[1], imag[1], real[2], imag[2] ...
  real[(N/2)-1], imag[(N/2)-1 }
  */
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

static inline void cacfar(float32_t * fftmag, float32_t * threshold, float32_t Pfa, int guard, int training){
  float32_t alpha = (N_SAMPLES/2)*(powf(Pfa,(float)-1/(N_SAMPLES/2)) - 1);
  float32_t suml = 0;
  float32_t sumr = 0;
  for(int i = (guard + training - 1); i < ((N_SAMPLES/2) - (guard + training)); ++i){ // i is the CUT (Cell Under Test)
    for(int j = i+guard, k = i-guard; j < (i + guard + training), k > (i - (guard + training)); ++j, --k){ // j sums the right side, k the left side
      suml += fftmag[k];
      sumr += fftmag[j];// |training cells | guard cells | CUT | guard cells | training cells|
    }
    threshold[i] = alpha*((suml+sumr)/(2*training));
    if(threshold[i] > fftmag[i]){
      fftmag[i] = 0;
    }
    suml = 0;
    sumr = 0;
  }
}
/* USER CODE END Application */

