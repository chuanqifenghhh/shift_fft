/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "Filter.h"
#include "head_define.h"
#include "stdio.h"
#include "base.h"
#include "Wifi.h" 
#include "MAX262.h"
#include "math.h"
#include "DDS.h"
#include "My_MATH.h"
#include  "DebugVisual.h"
#include <stdlib.h>
#include "uart_com.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DDS_N 8192
#define Is_TRI 0  //三角
#define Is_Sine 1  //正弦
#define reso 400
#define dds_clk 1000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
arm_cfft_radix4_instance_f32 scfft;
u32 out1_base,out2_base;
extern System_Amp_Fre Sys_1;
u16 AD_Cnt;
u16 AD_Value[AD_N];
float fre,max;
u32 max_index;
float
FFT_INPUT[AD_N],FFT_OUTPUT[AD_N];
u16 sine_wave[DDS_N];
u16 Out_1[DDS_N];
u16 trig_wave[DDS_N];
u16 Out_2[DDS_N];
extern u8 work_flag;
extern u8 out_flag;
volatile bool adc_finish = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Set_TIM_Freq(TIM_HandleTypeDef *htim, float freq) {
    uint32_t prescaler = 1;

    while ((float) SystemCoreClock / (float) prescaler / freq > 65535) {
        do {
            prescaler++;
        } while (SystemCoreClock % prescaler);
    }
    htim->Instance->PSC = prescaler - 1;
    htim->Instance->ARR = (uint16_t) ((float) SystemCoreClock / (float) prescaler / freq) - 1;
}







void SineWave_Data(uint16_t num, uint16_t* D, float U);
	
//void getAD_allch(float*result){
//	Get_AUTO_RST_Mode_Data((uint16_t*)ad_result,2);  //两个通道
//	*result=(ad_result[0]-ADS8688_ZERO)*AD_mV_Scale;
//	result+=AD_N;
//	*result=(ad_result[1]-ADS8688_ZERO)*AD_mV_Scale;
//	result+=AD_N;
//}

void AD_arr_Init(){
//	pAD_arr=AD_arr[0];pAD_arr_end=AD_arr[0]+AD_N;
//	memset(AD_arr[0],0,AD_N);
//	memset(AD_arr[1],0,AD_N);
	memset(AD_Value,0,AD_N);
	memset(FFT_INPUT,0,AD_N);
	memset(FFT_OUTPUT,0,AD_N);
	max_index = AD_Cnt = 0;
}



void sanWave_Data(uint16_t num, uint16_t* D, float U)             
{
    uint16_t i;
    u16 dd=U*4096.0f/3.3f;
    for (i = 0; i < num/2; i++)
    {
        D[i] = i*dd/(num/2);  
    }
    for (i = num/2; i < num; i++)
    {
        D[i] = D[num-i-1];
    }
}
void SineWave_Data(uint16_t num, uint16_t* D, float U)
{
	  num++;
    uint16_t i;
    for (i = 0; i < num-1; i++)
    {
        D[i] = (uint16_t)((U*sin((1.0 * i / (num - 1)) * 2 * 3.1415926) + U) * 4095 / 3.3)+300;    
    }
}
void SysInit()
{
	Sys_1.A_fre_word = Sys_1.B_fre_word  = 0 ;  //214533616;  //100K频率控制字
	Sys_1.square_cnt = Sys_1.A_Squr_fre  = 0 ;
	Sys_1.phase = 0;
	Sys_1.tim   = 0;
	AD_Cnt = 0;
}
void Get_AD(){
	u32 index1,index2;
	float max1,max2,amp1_max,amp2_max;
	float enge_A,enge_B;
	float windows[3288];
	hanning_window(windows,3288);

	AD_arr_Init();
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)AD_Value,AD_N);
	for(int i=0;i<AD_N;i++)
	{
		FFT_INPUT[i] = AD_Value[i]/4096.0*3300.0;
		
	}
	Mul_Windows(windows,FFT_INPUT,3288);               
	calc_FFT(FFT_INPUT,FFT_OUTPUT);
	arm_max_f32(&FFT_OUTPUT[10],AD_N/2,&max1,&index1);
	amp1_max = FFT_OUTPUT[index1+10];
	
	enge_A = FFT_OUTPUT[index1 + 10 - 3] + FFT_OUTPUT[index1 + 10 - 2] + FFT_OUTPUT[index1 +10 - 1]+FFT_OUTPUT[index1+10] + FFT_OUTPUT[index1+10 + 1]+FFT_OUTPUT[index1+10 + 2]+FFT_OUTPUT[index1+10 + 3];
	FFT_OUTPUT[index1 + 10] = 0;
	arm_max_f32(&FFT_OUTPUT[10],AD_N/2,&max2,&index2);
	amp2_max = FFT_OUTPUT[index2 +10];
	enge_B = FFT_OUTPUT[index2 +10 - 3] + FFT_OUTPUT[index2 +10 - 2]+FFT_OUTPUT[index2 +10 - 1]+FFT_OUTPUT[index2 +10] + FFT_OUTPUT[index2 +10 + 1]+FFT_OUTPUT[index2 + 10 + 2]+FFT_OUTPUT[index2 + 10 + 3];
	if(fabs(enge_A - enge_B) < 30.1f&&(enge_A + enge_B) > 180.0f )  //两个正弦幅度差别小
	{
		if(index1>index2) //index1对应频率为fB
		{
			Sys_1.B_kind = Sine;
			Sys_1.A_kind = Sine;
			Sys_1.B_fre_temp = reso*(index1 + 10);
			Sys_1.A_fre_temp = reso*(index2 + 10);
		}
		else 
		{	
			Sys_1.B_kind = Sine;
			Sys_1.A_kind = Sine;
			Sys_1.B_fre_temp = reso*(index2 + 10);
			Sys_1.A_fre_temp = reso*(index1 + 10);
			
		}
	}
	else if (fabs(amp1_max - amp2_max)<30.1f)
	{		
		if(index1>index2) //index1对应频率为fB
		{
			Sys_1.B_kind = Trig;
			Sys_1.A_kind = Trig;
			Sys_1.B_fre_temp = reso*(index1 + 10);
			Sys_1.A_fre_temp = reso*(index2 + 10);
		}
		else 
		{	
			Sys_1.B_kind = Trig;
			Sys_1.A_kind = Trig;
			Sys_1.B_fre_temp = reso*(index2 + 10);
			Sys_1.A_fre_temp = reso*(index1 + 10);
			
		}
	}
	else if(max1>max2)  //能量高的为正弦
	{
		if(index1>index2) //index1对应频率为fB
		{
			Sys_1.B_kind = Sine;
			Sys_1.A_kind = Trig;
			Sys_1.B_fre_temp = reso*(index1 + 10);
			Sys_1.A_fre_temp = reso*(index2 + 10);
		}
		else 
		{
			
			Sys_1.A_kind = Sine;
			Sys_1.B_kind = Trig;
			Sys_1.B_fre_temp = reso*(index2 + 10);
			Sys_1.A_fre_temp = reso*(index1 + 10);			
		}
	}
	else if(index1>index2) //index1对应频率为fB
		{
			Sys_1.B_kind =Trig ;
			Sys_1.A_kind = Sine;
			Sys_1.B_fre_temp = reso*(index1 + 10);
			Sys_1.A_fre_temp = reso*(index2 + 10);
		}
		else 
		{
			Sys_1.A_kind = Trig;
			Sys_1.B_kind = Sine;
			Sys_1.B_fre_temp = reso*(index2 + 10);
			Sys_1.A_fre_temp = reso*(index1 + 10);			
		}
			
		if(index1>index2)
		{
			Sys_1.B_fre_temp = reso*(index1 + 10);
			Sys_1.A_fre_temp = reso*(index2 + 10);
		}
		else {
			Sys_1.A_fre_temp = reso*(index1 + 10);
			Sys_1.B_fre_temp = reso*(index2 + 10);
		}
		
//		index1 = index1/10;
//		Sys_1.A_fre_temp = index1 * 400;
//		
//		index2 = index2/10;
//		Sys_1.B_fre_temp = index2 * 400;

		int temp1,temp2,temp3,temp4;
		int i,j,i2,j2;
		temp1 = temp2 = Sys_1.A_fre_temp;
		for(i=-15;i<0;i++)
			{
				temp1 = (temp1+100);
				if(temp1%5000==0)break;
		}
		for(j=0;j<15;j++)
			{
				temp2 = (temp2+100);
				if(temp2%5000==0)break;
		}
			if(fabs(16+i)<fabs(j))
				Sys_1.A_fre_temp = temp1 - 10;
			else Sys_1.A_fre_temp = temp2 -10;
			
		Sys_1.A_fre_word = 4294967296*Sys_1.A_fre_temp/2000000;		
		Sys_1.B_fre_word = 4294967296*Sys_1.B_fre_temp/2000000;
			
		temp3 = temp4 = Sys_1.B_fre_temp;
			for(i2=-15;i2<0;i2++)
			{
				temp3 = (temp3+100);
				if(temp3%5000==0)break;
		}
		for(j2=0;j2<15;j2++)
			{
				temp4 = (temp4+100);
				if(temp4%5000==0)break;
		}
			if(fabs(9+i2)<fabs(j2))
				Sys_1.B_fre_temp = temp3 - 10;
			else Sys_1.B_fre_temp = temp4 -10;
			Sys_1.B_fre_word = 4294967296*Sys_1.B_fre_temp/2000000;
}
			

void Out_Sign(){
//	HAL_TIM_Base_Start(&htim4);
//	if(Sys_1.A_kind == Sine && Sys_1.B_kind == Sine)
//	{
//		HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(u32*)sin_wave,DDS_N,DAC_ALIGN_12B_R);
//		HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,(u32*)sin_wave,DDS_N,DAC_ALIGN_12B_R);  
//	}
//	else if(Sys_1.A_kind == Sine && Sys_1.B_kind == Trig)
//	{
//			HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(u32*)sin_wave,DDS_N,DAC_ALIGN_12B_R);
//			HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,(u32*)tri_wave,DDS_N,DAC_ALIGN_12B_R);  
//	}
//	else if(Sys_1.A_kind == Trig && Sys_1.B_kind == Sine)
//	{
//			HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(u32*)tri_wave,DDS_N,DAC_ALIGN_12B_R);
//			HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,(u32*)sin_wave,DDS_N,DAC_ALIGN_12B_R);  
//	}
//	else if(Sys_1.A_kind == Trig && Sys_1.B_kind == Trig)
//	{
//			HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(u32*)tri_wave,DDS_N,DAC_ALIGN_12B_R);
//			HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,(u32*)tri_wave,DDS_N,DAC_ALIGN_12B_R);  
//	}
				HAL_TIM_Base_Start(&htim4);
				HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(u32*)Out_1,DDS_N,DAC_ALIGN_12B_R);
				HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,(u32*)Out_2,DDS_N,DAC_ALIGN_12B_R); 
	
}	
void Close_Sign()
{
	HAL_TIM_Base_Stop(&htim4);

}
void cal_fre_word()
{
	Sys_1.A_fre_word = 4294967296 * Sys_1.A_fre/2000000;
	Sys_1.B_fre_word = 4294967296 * Sys_1.A_fre/2000000;
}
void cal_fre(){
	
 HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);

}

void softReset()  //软件复位
{
	HAL_NVIC_SystemReset();
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  delay_init(168);
  //SysInit();
	myUART_Init(&huart6,&myUART1);
 // Wifi_Connect();	//wifi初始化 
	
  SineWave_Data(DDS_N,sine_wave,1.2);
	//sanWave_Data(DDS_N,trig_wave,1.2);

  AD_arr_Init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  arm_cfft_radix4_init_f32(&scfft,1024, 0, 1);
  Set_TIM_Freq(&htim3, 4000);
	int uart_flag = 1;
while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_TIM_Base_Start(&htim3);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)AD_Value,AD_N/2);
		while(adc_finish == false);
		/*
	   for (int i = 0; i < 2048; i++)
		{
     char buf[10];
     float adc_voltage = (float) AD_Value[i] / 4095 * 3.3f;
     int len = sprintf(buf, "%d,%.2f\n", i, adc_voltage);
     HAL_UART_Transmit(&huart6, (uint8_t *) buf, len, HAL_MAX_DELAY);
     }
		*/
		
		
		/*
		for(int i=0;i<AD_N;i++)
	  {
		FFT_INPUT[i] = AD_Value[i]/4096.0*3.3;
	  }
		calc_FFT(FFT_INPUT,FFT_OUTPUT);
		*/
		if(uart_flag)
				  {
		for(int i = 0; i < 1024; i++)
      {
          FFT_INPUT[2*i] = (float) AD_Value[i]; 
          FFT_INPUT[2*i+1] = 0;
      }
      arm_cfft_radix4_f32(&scfft, FFT_INPUT);
      arm_cmplx_mag_f32(FFT_INPUT,FFT_OUTPUT,1024);
		

		char buf[40],begin[40];
      int len,length;
			float fft_voltage;
			length = sprintf(begin, "page page1\xff\xff\xff");
      len = sprintf(buf, "add s0.id,0,0\xff\xff\xff");
			HAL_UART_Transmit(&huart6, (uint8_t *) begin, length, HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart6, (uint8_t *) buf, len, HAL_MAX_DELAY);
      for (int i = 0; i < 512; i++) {
          
          if (i == 0) {
              fft_voltage = FFT_OUTPUT[i] / 1024 / 4095 * 3.3f;
          } else {
              fft_voltage = FFT_OUTPUT[i] / 512 / 4095 * 3.3f;
          }
          
      }
			   
					 for (int i = 512; i > 0 ; i--) {
			      len = sprintf(buf, "add s0.id,0,%d\xff\xff\xff",(int)(FFT_OUTPUT[i] / 512 / 4095 * 3*1000));
            HAL_UART_Transmit(&huart6, (uint8_t *) buf, len, HAL_MAX_DELAY);
					 }
				 
		
		u32 index1,index2,index3;
		float max1,max2,max3;
		arm_max_f32(&FFT_OUTPUT[10],AD_N/4,&max1,&index1);
    FFT_OUTPUT[index1 + 10] = 0;FFT_OUTPUT[index1 + 9] = 0;FFT_OUTPUT[index1 + 8] = 0;FFT_OUTPUT[index1 + 11] = 0;FFT_OUTPUT[index1 + 12] = 0;FFT_OUTPUT[index1 + 13] = 0;FFT_OUTPUT[index1 + 7] = 0;
	  arm_max_f32(&FFT_OUTPUT[10],AD_N/4,&max2,&index2);
    FFT_OUTPUT[index2 + 10] = 0;FFT_OUTPUT[index2 + 9] = 0;FFT_OUTPUT[index2 + 8] = 0;FFT_OUTPUT[index2 + 11] = 0;FFT_OUTPUT[index2 + 12] = 0;FFT_OUTPUT[index2 + 13] = 0;FFT_OUTPUT[index2 + 7] = 0;
	  arm_max_f32(&FFT_OUTPUT[10],AD_N/4,&max1,&index3);
    FFT_OUTPUT[index3 + 10] = 0;FFT_OUTPUT[index3 + 9] = 0;FFT_OUTPUT[index3 + 8] = 0;FFT_OUTPUT[index3 + 11] = 0;FFT_OUTPUT[index3 + 12] = 0;FFT_OUTPUT[index3 + 13] = 0;FFT_OUTPUT[index3 + 7] = 0;
			char buf2[40],buf3[40],buf4[40],buf5[40];
			int len2,len3,len4,len5;
			len2 = sprintf(buf2,"page1.n0.val=%d\xff\xff\xff",index1*2+18);
			len3 = sprintf(buf3,"page1.n1.val=%d\xff\xff\xff",index2*2+10);
			len4 = sprintf(buf4,"page1.n2.val=%d\xff\xff\xff",index3*2+10);
			len5 = sprintf(buf5,"cle page1.s0.id,0\xff\xff\xff");
	
			
		 HAL_UART_Transmit(&huart6, (uint8_t *) buf2, len2, HAL_MAX_DELAY);
		 HAL_UART_Transmit(&huart6, (uint8_t *) buf3, len3, HAL_MAX_DELAY);
		 HAL_UART_Transmit(&huart6, (uint8_t *) buf4, len4, HAL_MAX_DELAY);
				 
		 //HAL_UART_Transmit(&huart6, (uint8_t *) buf5, len5, HAL_MAX_DELAY);

            }
					 uart_flag = 0;
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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


void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	if(Sys_1.A_kind == Sine)
		for(int i=0;i<DDS_N/2;i++ )
			{
				Out_1[i] = sine_wave[out1_base>>19];
				out1_base += Sys_1.A_fre_word;
			}
	else if(Sys_1.A_kind == Trig)
			for(int i=0;i<DDS_N/2;i++ )
			{		

				Out_1[i] = trig_wave[out1_base>>19];
				out1_base += Sys_1.A_fre_word;

			}

}
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	if(Sys_1.A_kind == Sine)
		for(int i=DDS_N/2;i<DDS_N;i++ )
			{
				Out_1[i] = sine_wave[out1_base>>19];
				out1_base += Sys_1.A_fre_word;
//			Out_2[i] = sine_wave[out2_base>>19];
//			out2_base += Sys_1.B_fre_word;
			}
		else if(Sys_1.A_kind == Trig)
			for(int i=DDS_N/2;i<DDS_N;i++ )
			{
				Out_1[i] = trig_wave[out1_base>>19];
				out1_base += Sys_1.A_fre_word;
//				Out_2[i] = sine_wave[out2_base>>19];
//				out2_base += Sys_1.B_fre_word;
			}

}



void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
	if(Sys_1.B_kind == Sine)
		for(int i=DDS_N/2;i<DDS_N;i++ )
			{
				Out_2[i] = sine_wave[out2_base>>19];
				out2_base += Sys_1.B_fre_word;
//			Out_2[i] = sine_wave[out2_base>>19];
//			out2_base += Sys_1.B_fre_word;
			}
		else if(Sys_1.B_kind == Trig)
			for(int i=DDS_N/2;i<DDS_N;i++ )
			{
				Out_2[i] = trig_wave[out2_base>>19];
				out2_base += Sys_1.B_fre_word;
//				Out_2[i] = sine_wave[out2_base>>19];
//				out2_base += Sys_1.B_fre_word;
			}
		}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
		if(Sys_1.B_kind == Sine)
		for(int i=0;i<DDS_N/2;i++ )
			{
				Out_2[i] = sine_wave[out2_base>>19];
				out2_base += Sys_1.B_fre_word;
				
			}
	else if(Sys_1.B_kind == Trig)
			for(int i=0;i<DDS_N/2;i++ )
			{
				Out_2[i] = trig_wave[out2_base>>19];
				out2_base += Sys_1.B_fre_word;
			}



}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	  adc_finish = true;
    HAL_TIM_Base_Stop(&htim3);
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
