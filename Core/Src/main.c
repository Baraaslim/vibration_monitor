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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "adxl345.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_SAMPLES 1024
#define FS_HZ 1000.0f   
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t samples[2][N_SAMPLES][3];   /* 2 buffers x 1000 samples x (X,Y,Z) */
static volatile uint16_t sample_idx = 0;   /* where the ISR writes next         */
static volatile uint8_t  fill_buf   = 0;   /* which buffer the ISR is filling   */
static volatile int8_t   ready_buf  = -1;  /* full buffer for main (-1 = none)  */
static volatile uint32_t overruns   = 0;   /* main too slow? count it           */

static adxl345_t accel = {
  .hspi    = &hspi2,
  .cs_port = ADXL_CS_GPIO_Port,
  .cs_pin  = ADXL_CS_Pin,
};

static arm_rfft_fast_instance_f32 fft;      /* FFT settings, prepared once */
static float32_t hann[N_SAMPLES];           /* window, computed once       */
static float32_t fft_in[N_SAMPLES];         /* windowed input              */
static float32_t fft_out[N_SAMPLES];        /* complex output              */
static float32_t fft_mag[N_SAMPLES / 2];    /* magnitude per bin           */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Send each printf character out of USART2 (to the PC) */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* RMS of each axis over one buffer of N_SAMPLES, in milli-g */
static void compute_rms(int16_t (*buf)[3], float rms_mg[3])
{
  for (int axis = 0; axis < 3; axis++)
  {
    float sum = 0.0f;
    for (int i = 0; i < N_SAMPLES; i++) sum += buf[i][axis];
    float mean = sum / N_SAMPLES;                   /* gravity + offset */

    float sum_sq = 0.0f;
    for (int i = 0; i < N_SAMPLES; i++)
    {
      float d = buf[i][axis] - mean;                /* only the wiggle */
      sum_sq += d * d;                              /* square it       */
    }
    rms_mg[axis] = sqrtf(sum_sq / N_SAMPLES) * 3.9f;  /* root of mean, in mg */
  }
}

/* FFT on one axis: biggest peak's frequency (Hz) and amplitude (mg) */
static void find_peak(int16_t (*buf)[3], int axis, float *peak_hz, float *peak_mg)
{
  /* 1. remove the average (gravity + offset) */
  float sum = 0.0f;
  for (int i = 0; i < N_SAMPLES; i++) sum += buf[i][axis];
  float mean = sum / N_SAMPLES;

  /* 2. apply the Hann window (fade the edges) */
  for (int i = 0; i < N_SAMPLES; i++)
    fft_in[i] = (buf[i][axis] - mean) * hann[i];

  /* 3. FFT: time domain -> frequency domain */
  arm_rfft_fast_f32(&fft, fft_in, fft_out, 0);

  /* 4. magnitude of each frequency bin */
  arm_cmplx_mag_f32(fft_out, fft_mag, N_SAMPLES / 2);
  fft_mag[0] = 0.0f;                                /* ignore 0 Hz */

  /* 5. biggest bin, skipping bins 0-1 (below ~2 Hz) */
  float32_t max_val;
  uint32_t  max_idx;
  arm_max_f32(&fft_mag[2], N_SAMPLES / 2 - 2, &max_val, &max_idx);
  max_idx += 2;

  /* 6. bin -> Hz, magnitude -> amplitude in mg */
  *peak_hz = max_idx * FS_HZ / N_SAMPLES;
  *peak_mg = max_val * 2.0f / (N_SAMPLES * 0.5f) * 3.9f;
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
   
  /* Prepare the FFT and the window once, before sampling starts */
  arm_rfft_fast_init_f32(&fft, N_SAMPLES);
  for (int i = 0; i < N_SAMPLES; i++)
    hann[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / (N_SAMPLES - 1)));

  if (adxl345_init(&accel))
  {
    printf("ADXL345 found. Starting...\r\n");
    HAL_TIM_Base_Start_IT(&htim2);
  }
  else
  {
    printf("ADXL345 not found! Check wiring.\r\n");
  }
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
  {
    if (ready_buf != -1)                       /* a full buffer is ready */
    {
      int8_t b = ready_buf;
      float rms[3], peak_hz, peak_mg;

      compute_rms(samples[b], rms);
      find_peak(samples[b], 0, &peak_hz, &peak_mg);   /* axis 0 = X */
      ready_buf = -1;                          /* buffer free again */

      int hz10 = (int)(peak_hz * 10.0f);       /* one decimal without %f */
      printf("RMS X=%3d Y=%3d Z=%3d mg | X peak: %d.%d Hz (%d RPM), %d mg\r\n",
             (int)rms[0], (int)rms[1], (int)rms[2],
             hz10 / 10, hz10 % 10, (int)(peak_hz * 60.0f), (int)peak_mg);

      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    /* USER CODE END WHILE */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2) return;          /* only react to TIM2 */

  int16_t *s = samples[fill_buf][sample_idx];
    adxl345_read_xyz(&accel, &s[0], &s[1], &s[2]);   /* ~30 us of SPI */
  sample_idx++;

  if (sample_idx >= N_SAMPLES)                 /* buffer full? */
  {
    if (ready_buf != -1) overruns++;           /* main didn't finish the last one */
    ready_buf  = fill_buf;                     /* hand this buffer to main */
    fill_buf  ^= 1;                            /* switch: 0 -> 1, 1 -> 0  */
    sample_idx = 0;
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
