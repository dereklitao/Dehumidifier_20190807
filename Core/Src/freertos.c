/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "csro_common.h"
#include "gpio.h"
#include "dac.h"
#include "adc.h"
#include "tim.h"

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
/* USER CODE BEGIN Variables */
uint32_t adc_data[100];
/* USER CODE END Variables */
osThreadId Task01Handle;
osThreadId Task02Handle;
osThreadId Task03Handle;
osThreadId Task04Handle;
osThreadId Task05Handle;
osThreadId Task06Handle;
osThreadId Task07Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Task01_cps_write_task(void const * argument);
void Task02_cps_read_task(void const * argument);
void Task03_hmi_wait_cmd_task(void const * argument);
void Task04_aqi_read_task(void const * argument);
void Task05_pwm_adc_dac_gpio_task(void const * argument);
void Task06_misc_func_task(void const * argument);
void Task07_stepper_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the thread(s) */
  /* definition and creation of Task01 */
  osThreadDef(Task01, Task01_cps_write_task, osPriorityHigh, 0, 256);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, Task02_cps_read_task, osPriorityAboveNormal, 0, 256);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* definition and creation of Task03 */
  osThreadDef(Task03, Task03_hmi_wait_cmd_task, osPriorityNormal, 0, 256);
  Task03Handle = osThreadCreate(osThread(Task03), NULL);

  /* definition and creation of Task04 */
  osThreadDef(Task04, Task04_aqi_read_task, osPriorityBelowNormal, 0, 256);
  Task04Handle = osThreadCreate(osThread(Task04), NULL);

  /* definition and creation of Task05 */
  osThreadDef(Task05, Task05_pwm_adc_dac_gpio_task, osPriorityLow, 0, 256);
  Task05Handle = osThreadCreate(osThread(Task05), NULL);

  /* definition and creation of Task06 */
  osThreadDef(Task06, Task06_misc_func_task, osPriorityIdle, 0, 256);
  Task06Handle = osThreadCreate(osThread(Task06), NULL);

  /* definition and creation of Task07 */
  osThreadDef(Task07, Task07_stepper_task, osPriorityBelowNormal, 0, 256);
  Task07Handle = osThreadCreate(osThread(Task07), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Task01_cps_write_task */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Task01_cps_write_task */
void Task01_cps_write_task(void const * argument)
{
    
    
    
    
    
    
    
    

  /* USER CODE BEGIN Task01_cps_write_task */
  csro_master_cps_init(&huart3);
  /* Infinite loop */
  for (;;)
  {
    csro_master_cps_write_task();
  }
  /* USER CODE END Task01_cps_write_task */
}

/* USER CODE BEGIN Header_Task02_cps_read_task */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task02_cps_read_task */
void Task02_cps_read_task(void const * argument)
{
  /* USER CODE BEGIN Task02_cps_read_task */
  /* Infinite loop */
  for (;;)
  {
    osDelay(200);
    csro_master_cps_read_task();
  }
  /* USER CODE END Task02_cps_read_task */
}

/* USER CODE BEGIN Header_Task03_hmi_wait_cmd_task */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task03_hmi_wait_cmd_task */
void Task03_hmi_wait_cmd_task(void const * argument)
{
  /* USER CODE BEGIN Task03_hmi_wait_cmd_task */
  csro_slave_hmi_init(&huart2);
  /* Infinite loop */
  for (;;)
  {
    csro_slave_hmi_wait_cmd();
  }
  /* USER CODE END Task03_hmi_wait_cmd_task */
}

/* USER CODE BEGIN Header_Task04_aqi_read_task */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task04_aqi_read_task */
void Task04_aqi_read_task(void const * argument)
{
  /* USER CODE BEGIN Task04_aqi_read_task */
  csro_master_aqi_init(&huart1);
  /* Infinite loop */
  for (;;)
  {
    osDelay(200);
    csro_master_aqi_read_task();
  }
  /* USER CODE END Task04_aqi_read_task */
}

/* USER CODE BEGIN Header_Task05_pwm_adc_dac_gpio_task */
/**
* @brief Function implementing the Task05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task05_pwm_adc_dac_gpio_task */
void Task05_pwm_adc_dac_gpio_task(void const * argument)
{
  /* USER CODE BEGIN Task05_pwm_adc_dac_gpio_task */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adc_data, 100);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  uint8_t adc_count = 0;
  /* Infinite loop */
  for (;;)
  {
    osDelay(50);

    TIM2->CCR1 = sys_regs.holdings[3];
    TIM2->CCR2 = sys_regs.holdings[2];
    TIM2->CCR3 = sys_regs.holdings[1];
    TIM2->CCR4 = sys_regs.holdings[0];
    sys_regs.discs[0] = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
    sys_regs.discs[1] = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
    sys_regs.discs[2] = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
    sys_regs.discs[3] = HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin);
    sys_regs.discs[4] = HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin);
    sys_regs.discs[5] = HAL_GPIO_ReadPin(SW6_GPIO_Port, SW6_Pin);
    HAL_GPIO_WritePin(SR1_GPIO_Port, SR1_Pin, sys_regs.coils[0] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR2_GPIO_Port, SR2_Pin, sys_regs.coils[1] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR3_GPIO_Port, SR3_Pin, sys_regs.coils[2] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR4_GPIO_Port, SR4_Pin, sys_regs.coils[3] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(DR1_GPIO_Port, DR1_Pin, sys_regs.coils[4] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(DR2_GPIO_Port, DR2_Pin, sys_regs.coils[5] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(DR3_GPIO_Port, DR3_Pin, sys_regs.coils[6] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(DR4_GPIO_Port, DR4_Pin, sys_regs.coils[7] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    adc_count = (adc_count + 1) % 4;
    if (adc_count == 1)
    {
      float sum[10] = {0};
      float voltage[10] = {0};
      float vadc = 0;
      for (uint8_t i = 0; i < 10; i++)
      {
        for (uint8_t j = 0; j < 10; j++)
        {
          sum[j] += adc_data[i * 10 + j];
        }
      }
      vadc = 1.225 * 4096.0 / (sum[9] / 10.0);
      for (uint8_t i = 0; i < 10; i++)
      {
        voltage[i] = (sum[i] / 10.0) * vadc / 4096.0;
      }
      for (uint8_t i = 0; i < 4; i++)
      {
        float ntc_res = voltage[3 - i] * 10.0 / (vadc - voltage[3 - i]);
        sys_regs.inputs[i] = (int16_t)(Csro_Calculate_ntc3950_Temperature_from_Resvalue(ntc_res) * 10.0);
      }
      for (uint8_t i = 0; i < 2; i++)
      {
        float rhi_temp_res = voltage[i * 2 + 5] * 10.0 / (vadc - voltage[i * 2 + 5]);
        sys_regs.inputs[i * 2 + 4] = (int16_t)(Csro_Calculate_ntc3380_Temperature_from_Resvalue(rhi_temp_res) * 10.0);
        sys_regs.inputs[i * 2 + 5] = (int16_t)(voltage[i * 2 + 4] / 3.3 * 100 * 10);
      }
    }
  }
  /* USER CODE END Task05_pwm_adc_dac_gpio_task */
}

/* USER CODE BEGIN Header_Task06_misc_func_task */
/**
* @brief Function implementing the Task06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task06_misc_func_task */
void Task06_misc_func_task(void const * argument)
{
  /* USER CODE BEGIN Task06_misc_func_task */
  /* Infinite loop */
  for (;;)
  {
    osDelay(500);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
  /* USER CODE END Task06_misc_func_task */
}

/* USER CODE BEGIN Header_Task07_stepper_task */
/**
* @brief Function implementing the Task07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task07_stepper_task */
void Task07_stepper_task(void const * argument)
{
  /* USER CODE BEGIN Task07_stepper_task */
  Csro_Stepper_Init();
  /* Infinite loop */
  for (;;)
  {
    osDelay(200);
    Csro_Stepper_Set_Position(sys_regs.holdings[4]);
  }
  /* USER CODE END Task07_stepper_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
