/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "adc.h"
#include "dac.h"
#include "usart.h"
#include "Delay.h"
#include "DDS.h"
#include "FFT.h"
#include "LCDAPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 50  // 50ms 消抖延时
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_dac1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern DDS_TypeDef DDS;
extern uint8_t ADCready;
extern uint16_t ADCbuff[FFT_SIZE];
extern uint8_t measureFlag;
extern uint32_t sampleRate;

extern DMA_HandleTypeDef hdma_dac1;
extern DMA_HandleTypeDef hdma_adc1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac1);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY5_Pin);
  HAL_GPIO_EXTI_IRQHandler(KEY4_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY3_Pin);
  HAL_GPIO_EXTI_IRQHandler(KEY2_Pin);
  HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
// 按键控制波形参数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	static uint32_t last_interrupt_time = 0;

    // 读取当前时间
    uint32_t current_time = HAL_GetTick();  // HAL_GetTick 返回系统时钟（毫秒）

    // 判断是否满足消抖时间（上次中断到现在超过 50ms）
    if (current_time - last_interrupt_time >= DEBOUNCE_DELAY) 
	{
		// 按键有效，开始处理
		
		// 按键1切换波形类型，切换顺序为正弦波->方波->三角波->矩形波
		// 三角波与矩形波占空比默认为 50%
		if(GPIO_Pin == KEY1_Pin)
		{
			if(DDS.waveType + 1 > 3)
				DDS.waveType = SINE_WAVE;
			else
			{
				DDS.waveType += 1;
				if(DDS.waveType == RECT_WAVE || DDS.waveType == TRIANGLE_WAVE)
					DDS.duty = 0.5f;
			}
		}
		// 按键2控制信号频率，频率范围100Hz~10000Hz，信号频率小于1kHz时，步进间隔为300Hz
		// 信号频率大于1000Hz时，步进间隔为1000Hz
		else if(GPIO_Pin == KEY2_Pin)
		{
			if(DDS.freq == 10000)
				DDS.freq = 100;	
			else if(DDS.freq < 1000)
				DDS.freq += 300;
			else
				DDS.freq += 1000;
		}
		// 按键3控制信号幅值，由于DAC输出误差，设定范围为1.0~2.2V，步进间隔为0.2V
		else if(GPIO_Pin == KEY3_Pin)
		{
			if(DDS.amp >= 2.2f)
				DDS.amp = 1.0f;
			else
				DDS.amp += 0.2f;
		}
		// 按键4控制占空比，只在波形为三角波和矩形波时生效
		// 占空比范围10%~90%，默认状态50%，步进间隔为10%
		else if(GPIO_Pin == KEY4_Pin)
		{
			if(DDS.waveType != TRIANGLE_WAVE && DDS.waveType != RECT_WAVE)
				return;
		
			if(DDS.duty >= 0.9f)
				DDS.duty = 0.1f;
			else
				DDS.duty += 0.1f;
		}
		// 按键5控制直流偏置电压，范围为0.2~1V，默认状态为0.2V，步进间隔为0.1V
		else if(GPIO_Pin == KEY5_Pin)
		{
			if(DDS.offset >= 1.0f)
				DDS.offset = 0.2f;
			else
				DDS.offset += 0.1f;
		}
		
		// 重启DDS
		measureFlag = 0;
		DDS_setWaveParams(DDS.freq, DDS.amp, DDS.waveType, DDS.duty, DDS.offset);
	
		LCD_FillScreen(LCD_COLOR_WHITE);
		LCD_Disp_Text(10, 130, LCD_COLOR_BLACK, 3, ASCII5x7, "MEASURING...");
	
		// 重新开启ADC测量
		TIM3->PSC = 0;
		TIM3->ARR = 840 - 1;
		sampleRate = 100000;
		HAL_TIM_Base_Start(&htim3);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCbuff, FFT_SIZE);
		
		// 更新最后中断时间
        last_interrupt_time = current_time;
	}
}

// DMA传输ADC数据完成
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		// 关闭ADC和时钟
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_TIM_Base_Stop(&htim3);
		
		ADCready = 1;
	}
}

/* USER CODE END 1 */
