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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Delay.h"
#include "LCDAPI.h"
#include "DDS.h"
#include "FFT.h"
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
extern DDS_TypeDef DDS;
extern float freamp[50];
extern uint16_t ADCbuff[FFT_SIZE];		// 采样数据
extern float fft_outputbuf[FFT_SIZE];	// 保存FFT后的幅值
extern float window[FFT_SIZE]; 			// 窗函数
extern float windowdata[FFT_SIZE]; 		// 窗函数处理后的数据
extern float biasVoltage;				// 直流偏置电压
extern float mainFreq;					// 基波频率
extern float Vopp;						// 峰峰值
extern float Duty;						// 占空比
extern uint16_t ADCbuff[FFT_SIZE];		// 采样数据
extern uint32_t sampleRate;				// 采样率
uint8_t ADCready = 0;
uint8_t measureFlag = 0;				// 频率测量精度标识符，0为粗测，1为精测
uint8_t WAVE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief       识别波形
 * @param       无
 * @retval      WAVE枚举类型{SINE_WAVE, SQUARE_WAVE, TRIANGLE_WAVE, RECT_WAVE}
 */
uint8_t identifyWaveType()
{
	float max = 0;
	float min = 4095;
	int i;
	uint32_t average = 0;
	
	for(i = 0; i < FFT_SIZE; ++i)
	{
		if(ADCbuff[i] > max)
			max = ADCbuff[i];
		if(ADCbuff[i] < min)
			min = ADCbuff[i];
		
		average += ADCbuff[i];
	}
	// 粗测得到直流偏置
	biasVoltage = (float)average / (float)FFT_SIZE;
	
	uint16_t mid = max - (max - min) / 4;
	// 首先识别方波/矩形波
	uint16_t mid_1 = max - 0.2f * (max - min);
	uint16_t mid_2 = min + 0.2f * (max - min);
	uint16_t high = 0;
	uint16_t low = 0;
	int temp_1 = 0;
	int temp_2 = 0;
	
	for(i = 0; i < FFT_SIZE; ++i)
	{
		if(mid_1 > ADCbuff[i] && ADCbuff[i] > mid_2)
			temp_1++;
		if(ADCbuff[i] > mid)
			temp_2++;
		
		// 顺便计算占空比
		if(ADCbuff[i] > biasVoltage)
			high++;
		if(ADCbuff[i] < biasVoltage)
			low++;
	}
	
	Duty = (float)high / (float)(high + low);
	
	// 留一些误差的余量
	if(temp_1 < 50)
	{
		// 分析占空比
		if(0.485f < Duty && Duty < 0.515f)
			return SQUARE_WAVE;
		else
			return RECT_WAVE;
	}
	
	// 判断正弦波与三角波
	float ratio = (float)temp_2 / (float)FFT_SIZE;
	if(ratio > 0.30f)
		return SINE_WAVE;
	else 
		return TRIANGLE_WAVE;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int i;		// 计数器
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  delay_init(168);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  
  // DDS初始状态：幅值为1.0V，频率为100Hz的正弦波，直流偏置为0.2V
  DDS.amp = 1.0;
  DDS.freq = 100;
  DDS.duty = 0.5;
  DDS.waveType = SINE_WAVE;
  DDS.offset = 0.2;
  DDS_Start();
  
  printf("start\r\n");
  
  LCD_Disp_Text(10, 130, LCD_COLOR_BLACK, 3, ASCII5x7, "MEASURING...");
  
//  LCD_Disp_Text(10, 10, LCD_COLOR_BLACK, 3, ASCII5x7, "Succeed!");
//  LCD_Disp_Text(15, 90, LCD_COLOR_BLACK, 2, ASCII5x7, "WAVE: ");
//  LCD_Disp_Text(15, 120, LCD_COLOR_BLACK, 2, ASCII5x7, "FREQ: ");
//  LCD_Disp_Text(15, 150, LCD_COLOR_BLACK, 2, ASCII5x7, "Vopp: ");
//  LCD_Disp_Text(15, 180, LCD_COLOR_BLACK, 2, ASCII5x7, "Offset: ");
//  LCD_Disp_Text(15, 210, LCD_COLOR_BLACK, 2, ASCII5x7, "Duty: ");
  
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCbuff, FFT_SIZE);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
	if(ADCready)
	{
		ADCready = 0;	
		
		// 粗测频率
		TIM3->PSC = 0;
		TIM3->ARR = 840 - 1;
		FFT_start();		
		
		if(!measureFlag)
		{
			// 粗测时采样频率较高，对波形一个周期内采样点数较多，计算峰峰值更准确
			// 因此先计算峰峰值再精测频率
			WAVE = identifyWaveType();
			
			float max = 0;
			float min = 4096;
			for(i = 0; i < FFT_SIZE; ++i)
			{
				if(ADCbuff[i] > max)
					max = ADCbuff[i];
		
				// ADCbuff[i] > 150 是因为dac输出给了一个 0.2V 电压的抬升，来保证波形不失真
				if(ADCbuff[i] < min && ADCbuff[i] > 150)
					min = ADCbuff[i];
			}
			Vopp = ((float)max - (float)min) / 4095.0f * 3.3f;
			
			
			if(mainFreq > 5000.0f)
			{
				measureFlag = 2;
				sampleRate = 100000;
			}
			else if(mainFreq > 1000.0f)
			{
				measureFlag = 1;
				TIM3->ARR = 4200 - 1;
				sampleRate = 20000;
			}
			else if(mainFreq > 500.0f)
			{
				measureFlag = 1;
				TIM3->ARR = 8400 - 1;
				TIM3->PSC = 2 - 1;
				sampleRate = 5000;
			}
			else
			{
				measureFlag = 1;
				TIM3->ARR = 2000 - 1;
				TIM3->PSC = 21 - 1;
				sampleRate = 2000;
			}
		}
		
		// 精测后运行以下代码
		if(measureFlag == 2)
		{
//			for(i = 0; i < FFT_SIZE; ++i)
//			{
//				printf("%.2f, ", fft_outputbuf[i]);
//			}
					
			// 输出数据
			FFT_getValue();
			
			LCD_FillScreen(LCD_COLOR_WHITE);
			LCD_Disp_Text(10, 10, LCD_COLOR_BLACK, 3, ASCII5x7, "Succeed!");
			LCD_Disp_Text(15, 90, LCD_COLOR_BLACK, 2, ASCII5x7, "WAVE: ");
			LCD_Disp_Text(15, 120, LCD_COLOR_BLACK, 2, ASCII5x7, "FREQ: ");
			LCD_Disp_Text(15, 150, LCD_COLOR_BLACK, 2, ASCII5x7, "Vopp: ");
			LCD_Disp_Text(15, 180, LCD_COLOR_BLACK, 2, ASCII5x7, "Offset: ");
			LCD_Disp_Text(15, 210, LCD_COLOR_BLACK, 2, ASCII5x7, "Duty: ");
			
			switch(WAVE){
				case SINE_WAVE: 
					LCD_Disp_Text(85, 90, LCD_COLOR_BLACK, 2, ASCII5x7, "Sine wave");
					LCD_Disp_Text(85, 210, LCD_COLOR_BLACK, 2, ASCII5x7, "NONE");
					break;
				case SQUARE_WAVE:
					LCD_Disp_Text(85, 90, LCD_COLOR_BLACK, 2, ASCII5x7, "Square wave");
					LCD_Disp_Decimal(85, 210, LCD_COLOR_BLACK, 2, ASCII5x7, (double)Duty * 100, 2, 2);
					LCD_Draw_Char(150, 210, LCD_COLOR_BLACK, 2, ASCII5x7, '%');
					break;
				case TRIANGLE_WAVE: 
					LCD_Disp_Text(85, 90, LCD_COLOR_BLACK, 2, ASCII5x7, "Triangle wave");
					LCD_Disp_Text(85, 210, LCD_COLOR_BLACK, 2, ASCII5x7, "Not measure");
					break;
				case RECT_WAVE:
					LCD_Disp_Text(85, 90, LCD_COLOR_BLACK, 2, ASCII5x7, "Rect wave");
					LCD_Disp_Decimal(85, 210, LCD_COLOR_BLACK, 2, ASCII5x7, (double)Duty * 100, 2, 2);
					LCD_Draw_Char(150, 210, LCD_COLOR_BLACK, 2, ASCII5x7, '%');
					break;
				default: break;
			}
			
			// 判断频率整数位数
			int intnum = 0;
			int temp = (int)mainFreq;
			while(temp > 0)
			{
				intnum++;
				temp /= 10;
			}
			
			LCD_Disp_Decimal(85, 120, LCD_COLOR_BLACK, 2, ASCII5x7, (double)mainFreq, intnum, 2);
			LCD_Disp_Text(130 + intnum * 10, 120, LCD_COLOR_BLACK, 2, ASCII5x7, "Hz");

			LCD_Disp_Decimal(85, 150, LCD_COLOR_BLACK, 2, ASCII5x7, (double)Vopp, 1, 2);
			LCD_Disp_Text(140, 150, LCD_COLOR_BLACK, 2, ASCII5x7, "V");
			
			LCD_Disp_Decimal(105, 180, LCD_COLOR_BLACK, 2, ASCII5x7, (double)biasVoltage, 1, 2);
			LCD_Disp_Text(160, 180, LCD_COLOR_BLACK, 2, ASCII5x7, "V");
			
			// 重置测量标识符
			measureFlag = 0;
		}
		// 开启精测模式
		else if(measureFlag == 1)
		{
			LCD_FillScreen(LCD_COLOR_WHITE);
			LCD_Disp_Text(10, 130, LCD_COLOR_BLACK, 3, ASCII5x7, "MEASURING...");
			measureFlag = 2;
			
			HAL_TIM_Base_Start(&htim3);
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCbuff, FFT_SIZE);
		}
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
