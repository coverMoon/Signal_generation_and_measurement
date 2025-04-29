#include "main.h"
#include "FFT.h"
#include "DDS.h"

uint32_t sampleRate = 100000;			// 初始采样率
uint16_t ADCbuff[FFT_SIZE];				// 采样数据
float fft_inputbuf[FFT_SIZE * 2];  		// 用于FFT的复数输入数据
float fft_outputbuf[FFT_SIZE];			// 保存FFT后的幅值
float freamp[50];						// 用于存放各次谐波频率和幅值的数组
float window[FFT_SIZE]; 				// 窗函数
float windowdata[FFT_SIZE]; 			// 窗函数处理后的数据
float biasVoltage;						// 直流偏置电压
float mainFreq;							// 基波频率
float Vopp;								// 峰峰值
float Duty;								// 占空比

extern uint8_t WAVE;

/**
 * @brief       开启FFT
 * @note		包括数据处理、加窗函数、寻找基频
 * @param       无
 * @retval      无
 */
void FFT_start()
{
	int i;						// 计数器
		
//	// 生成汉宁窗并应用到数据
//	for (int i = 0; i < FFT_SIZE; i++) 
//	{
//		window[i] = 0.5 * (1 - cos(2 * M_PI * i / (FFT_SIZE - 1)));  // 汉宁窗公式
//		windowdata[i] = ((float)ADCbuff[i] * 3.3f / 4096.0f) * window[i];  // 数据与窗函数相乘
//	}	
	
//	// 生成汉明窗并应用到数据
//	for (int i = 0; i < FFT_SIZE; i++) 
//	{
//		// 汉明窗公式
//		window[i] = 0.54 - 0.46 * cos(2 * M_PI * i / (FFT_SIZE - 1));  // 汉明窗公式
//    
//		// 数据与窗函数相乘
//		windowdata[i] = ((float)ADCbuff[i] * 3.3f / 4096.0f) * window[i];  // ADC数据与汉明窗相乘并转换到浮点
//	}
	
//	// 生成布莱克曼窗并应用到数据
//	for (int i = 0; i < FFT_SIZE; i++) 
//	{
//		// 布莱克曼窗公式
//		window[i] = 0.42 - 0.5 * cos(2 * M_PI * i / (FFT_SIZE - 1)) + 0.08 * cos(4 * M_PI * i / (FFT_SIZE - 1));  // 布莱克曼窗公式
//    
//		// 数据与窗函数相乘
//		windowdata[i] = ((float)ADCbuff[i] * 3.3f / 4096.0f) * window[i];  // ADC数据与布莱克曼窗相乘并转换到浮点
//	}

	// 生成布莱克曼-哈里斯窗并应用到数据
	for (int i = 0; i < FFT_SIZE; i++) 
	{
		// 布莱克曼-哈里斯窗公式
		window[i] = 0.35875 - 0.48829 * cos(2 * M_PI * i / (FFT_SIZE - 1)) + 0.14128 * cos(4 * M_PI * i / (FFT_SIZE - 1)) - 0.01168 * cos(6 * M_PI * i / (FFT_SIZE - 1));
    
		// 数据与窗函数相乘
		windowdata[i] = ((float)ADCbuff[i] * 3.3f / 4096.0f) * window[i];  // ADC数据与布莱克曼-哈里斯窗相乘并转换到浮点
	}


	// 初始化FFT输入数组
	for(i = 0; i < FFT_SIZE; ++i)
	{
		fft_inputbuf[i * 2] = (float)ADCbuff[i] * 3.3f / 4096.0f;	// 采样值填入实部部分
		fft_inputbuf[i * 2 + 1] = 0;			// 虚部全部为0
	}
				
	arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_inputbuf, 0, 1);  	//fft运算
	arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_SIZE);			//把运算结果复数求模得幅值
		
	int temp_num = 1;
	float temp_value = fft_outputbuf[1];
	
	if(WAVE == RECT_WAVE && (Duty > 0.7f || Duty < 0.3f))
	{
		// 矩形波在占空比较大或较小时，使用窗函数处理易造成谐波增强，因此需要进行特殊处理
		int upper = 0;
		if(mainFreq > 5000)
			upper = 450;
		else if(mainFreq > 1000)
			upper = 1050;
		else if(mainFreq > 500)
			upper = 850;
		else
			upper = 1050;
		
		for(i = 2; i < upper; ++i)
		{
			if(fft_outputbuf[i] > temp_value)
			{
				temp_num = i;
				temp_value = fft_outputbuf[i];
			}
		}
	}
	else
	{
		for(i = 2; i < FFT_SIZE / 2; ++i)
		{
			if(fft_outputbuf[i] > temp_value)
			{
				temp_num = i;
				temp_value = fft_outputbuf[i];
			}
		}
	}
	
	mainFreq = (float)sampleRate / (float)FFT_SIZE * (float)temp_num;
}

/**
 * @brief       FFT成功后从输出数据中得到有用信息
 * @note		得到直流偏置电压、各次谐波等
 * @param       无
 * @retval      无
 */
void FFT_getValue(void)  
{
	int i;
	
//	float max = 0;
//	float min = 4096;
//	for(i = 0; i < FFT_SIZE; ++i)
//	{
//		if(ADCbuff[i] > max)
//			max = ADCbuff[i];
//		
//		// ADCbuff[i] > 150 是因为dac输出给了一个 0.2V 电压的抬升，来保证波形不失真
//		if(ADCbuff[i] < min && ADCbuff[i] > 150)
//			min = ADCbuff[i];
//	}
//	
//	// 得到峰峰值
//	Vopp = ((float)max - (float)min) / 4095.0f * 3.3f;
	
	// 得到直流偏置电压
	biasVoltage = fft_outputbuf[0] / 4096.0f;
	
	
	// 得到各次谐波
	for(i = 1; i < FFT_SIZE / 2; ++i)
	{
		// 只统计模值大于100的
		if(fft_outputbuf[i] > 100)
			freamp[i] = (float)sampleRate / (float)FFT_SIZE * fft_outputbuf[i];
	}
}
