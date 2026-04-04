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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
uint32_t previous_us_tick = 0;
volatile uint32_t distance;
volatile uint8_t data_ready = 0;
uint32_t start_time = 0, end_time = 0;
uint8_t is_first_captured = 0;
uint8_t sample_count = 0; // Đếm số lượng mẫu đã lấy
uint32_t distance_list[5] = {0};

uint32_t final_distance = 999;
uint8_t is_beeping = 0; // 0 là tắt, 1 là mở
uint32_t buzzer_last_toggle = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sonic_Sensor_Filter(void);
void buzzer_Beep(uint32_t now_Time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DWT_Init(void) {
  // Kích hoạt DWT (Trace Enable) trong thanh ghi DEMCR của CoreDebug
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Reset bộ đếm chu kỳ về 0
  DWT->CYCCNT = 0;

  // Bật bộ đếm (Cycle count enable) trong thanh ghi CTRL của DWT
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us) {
  uint32_t startTick = DWT->CYCCNT;
  uint32_t delayTicks = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - startTick) < delayTicks);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    if (is_first_captured == 0){ // Nếu là cạnh lên
      start_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      is_first_captured = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // Bật cạnh xuống
    }
    else{ // Nếu là cạnh xuống
      end_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

      if(end_time > start_time)
        distance = (end_time - start_time) / 58;
      else{
        distance = ((65535 - start_time) + end_time) / 58;
      }

      data_ready = 1;
      is_first_captured = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // Bật cạnh lên
    }
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); 

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  DWT_Delay_us(10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  uint32_t now = 0, last_change = 0;
  uint8_t index = 0;


  while (1)
  {
    now = HAL_GetTick();

    // ========================================================
    // TASK 1: ĐỌC VÀ LỌC CẢM BIẾN (Chạy mỗi 100ms)
    // ========================================================
    if (now - last_change >= 100) {
      if (data_ready == 1){
        data_ready = 0;
        // Cập nhật mảng data
        distance_list[index] = distance;
        index = (index + 1) % 5;

        sonic_Sensor_Filter();
      }
      else{
        // Lỗi mất kết nối
        if(sample_count > 0){
          printf("Sensor Timeout / Disconnected!\r\n");
          sample_count = 0;
          final_distance = 999;
        }
      }
      last_change = now;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
      DWT_Delay_us(10);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    }
    // ========================================================
    // TASK 2: ĐIỀU KHIỂN CÒI BÁO (Chạy liên tục không bị nghẽn)
    // ========================================================
    buzzer_Beep(now);
    
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
int _write(int fd, char* ptr, int len){
  HAL_StatusTypeDef hstatus;
  if(fd == 1 || fd == 2){
    hstatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK) return len;
  }
  return -1;
}

void sonic_Sensor_Filter(void){
if (sample_count < 5) {
          sample_count++;
        }
        // Tính toán bộ lọc
        if (sample_count == 5) {
          uint32_t sum = 0;
          uint32_t top = 0; 
          uint32_t bot = distance_list[0];

          for (int i = 0; i < 5; i++) {
            sum += distance_list[i];
            if (distance_list[i] > top) top = distance_list[i];
            if (distance_list[i] < bot) bot = distance_list[i];
          }
          sum -= (top + bot);
          final_distance = sum / 3;

          printf("Raw: %lu cm | Filtered: %lu cm\r\n", distance, final_distance);
        } else {
          printf("Raw: %lu cm | Loading filter...\r\n", distance);
        }
}

void buzzer_Beep(uint32_t now_Time){
if (sample_count < 5 || final_distance > 100) {
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      is_beeping = 0;
    }
    else if (final_distance <= 30) {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
      is_beeping = 1;
    }
    else{
      uint32_t beep_off_time = (final_distance * 5) + 50;
      if(is_beeping == 1){
        // Nếu còi đang kêu, kiểm tra xem đã kêu đủ 50ms chưa để tắt
        if(now_Time - buzzer_last_toggle >= 50){
          HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
          is_beeping = 0;
          buzzer_last_toggle = now_Time;
        }
      }
      else{
        if(now_Time - buzzer_last_toggle >= beep_off_time){ // Nếu còi đang tắt, kiểm tra xem đã nghỉ đủ chưa để bật
          HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
          is_beeping = 1; // Chuyển trạng thái sang kêu
          buzzer_last_toggle = now_Time;
        }
      }
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