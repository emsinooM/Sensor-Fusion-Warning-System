/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Chương trình Dung hợp dữ liệu Siêu âm và ToF (Sensor Fusion)
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
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "VL53L0X.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL53L0X_I2C_ADDRESS (0x29 << 1)
#define OUT_OF_RANGE 999
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t previous_us_tick = 0;

// --- Biến Siêu âm ---
volatile uint32_t distance_raw;
volatile uint8_t data_ready = 0;
uint32_t start_time = 0, end_time = 0;
uint8_t is_first_captured = 0;
uint8_t sample_count = 0; // Đếm số lượng mẫu đã lấy
uint32_t distance_list[5] = {0};
uint32_t sonic_final_cm = OUT_OF_RANGE;

// --- Biến ToF ---
struct VL53L0X myToF;
uint16_t tof_raw_distance = 0;

// --- Biến Hệ Thống (Sau khi dung hợp) ---
uint32_t system_distance_cm = OUT_OF_RANGE; // Khoảng cách cuối cùng được chốt

// --- Biến Còi ---
uint8_t is_beeping = 0; // 0 là tắt, 1 là mở
uint32_t buzzer_last_toggle = 0;

uint32_t final_distance = 999;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void sonic_Sensor_Filter(void);
uint32_t sensor_Fusion(uint32_t sonic_cm, uint32_t tof_mm);
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
        distance_raw = (end_time - start_time) / 58;
      else{
        distance_raw = ((65535 - start_time) + end_time) / 58;
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

  myToF.address = VL53L0X_I2C_ADDRESS;
  myToF.io_timeout = 500;

  printf("--- BAT DAU TEST DO KHOANG CACH VL53L0X ---\r\n");

  // uint8_t start_cmd = 0x01; // Lệnh 0x01 (Single-shot)
  // uint8_t status = 0;
  // uint8_t dist_data[2] = {0, 0};
  // uint16_t tof_distance_mm = 0;

  printf("Dang khoi tao vl53l0x...\r\n");
  if(VL53L0X_init(&myToF)){
    printf("Da khoi tao vl53l0x thanh cong...\r\n");
    // Cấu hình thời gian đo 33ms (đảm bảo tốc độ nhanh để chạy realtime)
    VL53L0X_setMeasurementTimingBudget(&myToF, 33000);
  }
  else{
    printf("Khoi tao vl53l0x that bai...\r\n");
  }

  // Trigger mồi hệ thống siêu âm
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  DWT_Delay_us(10);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
  

  // // 1. Gửi lệnh yêu cầu bắn laser vào thanh ghi SYSRANGE_START (0x00)
  // HAL_I2C_Mem_Write(&hi2c1, VL53L0X_I2C_ADDRESS, 0x00, 1, &start_cmd, 1, 100);
  // printf("-> Da ra lenh ban Laser...\r\n");

  // // 2. Vòng lặp chờ cảm biến đo xong
  // uint32_t timeout_tick = HAL_GetTick();
  // while((status & 0x01) == 0){
  //   HAL_I2C_Mem_Read(&hi2c1, VL53L0X_I2C_ADDRESS, 0x14, 1, &status, 1, 100);
  //   if(HAL_GetTick() - timeout_tick > 1000){
  //     printf("-> Timed out!\r\n");
  //     break;
  //   }
  // }

  // // 3. Nếu đo thành công, đọc 2 byte kết quả
  // if (status & 0x01){
  //   // Khoảng cách được lưu ở thanh ghi 0x1E (Byte cao) và 0x1F (Byte thấp)
  //   HAL_I2C_Mem_Read(&hi2c1, VL53L0X_I2C_ADDRESS, 0x1E, 1, dist_data, 2, 100);

  //   // Ghép 2 byte lại thành 1 số nguyên 16-bit
  //   tof_distance_mm = (dist_data[0] << 8) | dist_data[1];
  //   printf("-> Khoang cach: %d mm\r\n", tof_distance_mm);
  // }
  // printf("--------------------------------\r\n");
  // HAL_Delay(2000);


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
        distance_list[index] = distance_raw;
        index = (index + 1) % 5;

        sonic_Sensor_Filter();
      }
      else{
        // Lỗi mất kết nối
        if(sample_count > 0){
          printf("Sensor Timeout / Disconnected!\r\n");
          sample_count = 0;
          sonic_final_cm = OUT_OF_RANGE;
        }
      }
      tof_raw_distance = VL53L0X_readRangeSingleMillimeters(&myToF);

      if(sample_count == 5){
        system_distance_cm = sensor_Fusion(sonic_final_cm, tof_raw_distance);
        // In log tổng hợp ra màn hình
          printf("Sonic: %3lu cm | ToF: %3u cm | FINAL: %3lu cm\r\n", 
                 sonic_final_cm, (tof_raw_distance/10), system_distance_cm);
      }
      else{
        printf("Loading Filter...\r\n");
      }

      // printf("Sieu am: %lu cm | ToF: %u mm (~%u cm)\r\n", final_distance, tof_raw_distance, tof_raw_distance / 10);

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
          sonic_final_cm = sum / 3;
        }
}

uint32_t sensor_Fusion(uint32_t sonic_cm, uint32_t tof_mm){
  uint32_t tof_cm = tof_mm / 10;
  uint32_t fusion_cm = OUT_OF_RANGE;

  uint8_t sonic_valid = (sonic_cm > 0 && sonic_cm < 400);
  uint8_t tof_valid = (tof_mm > 0 && tof_mm < 2000);

  // TH1: Cả 2 đều bắt được vật thê
  if(sonic_valid && tof_valid){
    if (abs((uint32_t)sonic_cm) - abs((uint32_t)tof_cm) > 20){
      // Nếu chênh lệch nhiều -> Gặp vật thể đặc biệt
      // Ưu tiên khoảng cách nhỏ hơn
      fusion_cm = (sonic_cm < tof_cm)? sonic_cm : tof_cm;
      printf("[FUSION] Canh bao vat lieu dac biet! Chon: %lu cm\r\n", fusion_cm);
    }
    else{
      // Nếu không chênh lệch nhiều, ưu tiên tof
      fusion_cm = tof_cm;
    }
  }
  // TH2: ToF chết/ ngoài tầm, Sonic sống (Vật ở xa hoặc ngoài nắng)
  else if(sonic_valid && !tof_valid){
    fusion_cm = sonic_cm;
  }

  //TH3: Sonic chết, ToF sống (Bề mặt hút âm thanh, vật nghiêng)
  else if(!sonic_valid && tof_valid){
    fusion_cm = tof_cm;
  }

  //TH4: 2 cái chết
  else{
    fusion_cm = OUT_OF_RANGE;
  }
  return fusion_cm;
}

void buzzer_Beep(uint32_t now_Time){
if (sample_count < 5 || system_distance_cm > 100) {
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      is_beeping = 0;
    }
    else if (system_distance_cm <= 30) {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
      is_beeping = 1;
    }
    else{
      uint32_t beep_off_time = (system_distance_cm * 5) + 50;
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
