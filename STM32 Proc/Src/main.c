/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "elrs.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
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

extern DMA_HandleTypeDef hdma_usart1_rx;
uint8_t Rx_data[200] = {0};					//串口通信缓存
extern ELRS_Data elrs_data;					//接收机数据存储位置
float AngleData=0;							//舵机角度
//串口输出测试用数据：
uint8_t timetime=0;
char message_string[100]={0};


//下面的数据定义忘了是在哪了，没敢删 :(
char received_string[256];
char message[10]={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void floatToString(float num, char* str, int precision)
{
    sprintf(str, "%.*f", precision, num);
    HAL_Delay(10);
}

//串口接收事件返回函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
     if(huart==&huart2)
     {
		  //在回调函数中，(huart2)接收到的数据首先存储在 elrs_data_temp 数组中，然后根据不同的帧类型，提取特定的数据并映射到 elrs_data 结构体中
		  //float_Map 和 float_Map_with_median 函数用于将原始的 RC 通道数据转换为符合控制器要求的格式。具体来说：
		  //float_Map 将一个输入值从一个范围映射到另一个范围。
		  //float_Map_with_median 在映射时考虑了一个中位数，使得数据在中位数处有一个平滑过渡，避免了极端值的影响。

    	 ELRS_UARTE_RxCallback(Size);
     }
}


//控制相关函数

//设置舵机（左）角度    2024年12月9日01:04:46，将强制类型转化"(uint8_t)(50+200*ang/180)"移到外部，尝试解决舵机抖动的问题
void SevorAngleSet_L(float angle)
{
	int ang=angle;
	ang=50+200*ang/180;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ang);
}
//设置舵机（右）角度
void SevorAngleSet_R(float angle)
{
	int ang=angle;
	ang=50+200*(180-angle)/180;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ang);
}

//设置推进器端口电压值（左）TIM3-CH3   (TIM3   500Hz 寄存器值范围在0~1999 )
//测试时使用LED(呼吸灯)
void Thruster_L(float volt)
{
	int vol=volt;
	uint8_t v=(uint8_t)vol;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,(333*v));
}
//设置推进器端口电压值（右）TIM3-CH4
void Thruster_R(float volt)
{
	int vol=volt;
	uint8_t v=(uint8_t)vol;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,(333*v));

}
//偏航测试用LED函数     偏航、俯仰部分姑且先做成不安装jy901s的情况(偏航时根据左摇杆X轴改变输出电压)，后面再说
void Yaw_LED(void)
{
    //左摇杆上下油门，左右转向
    if(elrs_data.Left_Y>50&&elrs_data.Left_X<-30)
    {
   	 HAL_GPIO_WritePin(THRUSTER_R_GPIO_Port, THRUSTER_R_Pin,GPIO_PIN_SET );
   	 HAL_GPIO_WritePin(THRUSTER_L_GPIO_Port, THRUSTER_L_Pin,GPIO_PIN_RESET );
    }else if(elrs_data.Left_Y>50&&elrs_data.Left_X>30)
    {
   	 HAL_GPIO_WritePin(THRUSTER_R_GPIO_Port, THRUSTER_R_Pin,GPIO_PIN_RESET );
   	 HAL_GPIO_WritePin(THRUSTER_L_GPIO_Port, THRUSTER_L_Pin,GPIO_PIN_SET );
    }else  if(elrs_data.Left_Y>50)
    {
   	 HAL_GPIO_WritePin(THRUSTER_R_GPIO_Port, THRUSTER_R_Pin,GPIO_PIN_SET );
        HAL_GPIO_WritePin(THRUSTER_L_GPIO_Port, THRUSTER_L_Pin,GPIO_PIN_SET );
    }else
    {
   	 HAL_GPIO_WritePin(THRUSTER_R_GPIO_Port, THRUSTER_R_Pin,GPIO_PIN_RESET );
        HAL_GPIO_WritePin(THRUSTER_L_GPIO_Port, THRUSTER_L_Pin,GPIO_PIN_RESET );
    }
}
void Yaw_Thruster(void)
{
	float YawData=elrs_data.Left_X;
	float YawCoefficient = 1 - (YawData / 100);
	YawCoefficient += (YawData < 0) ? 2 * (YawData / 100) : 0;
	float ThrusterData=elrs_data.Left_Y;
	float Volt=ThrusterData/100*6;
	if(YawData<0)
	{
		Thruster_L(Volt*YawCoefficient);
		Thruster_R(Volt);
	}else
	{
		Thruster_L(Volt);
		Thruster_R(Volt*YawCoefficient);
	}

}
//重定义TIM结束返回函数，控制舵机
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
       if(htim==&htim4)//每0.1s进入一次中断
       {
    	   while(elrs_data.A!=0)//检测按键A是否按下，如果按下则将一切归位，指示灯闪烁，直到A抬起
    	   {
    		   HAL_GPIO_TogglePin(THRUSTER_R_GPIO_Port, THRUSTER_R_Pin);
    		   HAL_GPIO_TogglePin(THRUSTER_L_GPIO_Port, THRUSTER_L_Pin);
    		   Thruster_L(0);
    		   Thruster_R(0);
    		   SevorAngleSet_L(90);
			   SevorAngleSet_R(90);
    		   ELRS_Init();
    	   }
           //串口1输出测试部分：不用的时候可以注释掉
           //每一秒通过串口1发送一次数据
//           if(++timetime==10)
//           {
//        	   sprintf(message_string,"左_X轴：%.1f /n左_Y轴：%.1f /n 右_X轴：%.1f /n 右_Y轴：%.1f",elrs_data.Left_X,elrs_data.Left_Y,elrs_data.Right_X,elrs_data.Right_Y);
//        	   HAL_UART_Transmit_DMA(&huart1, (uint8_t*)message_string, strlen(message_string));
//        	   timetime=0;
//           }
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_data, 200 - 1);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  //初始化 huart2 的 DMA 接收,接收的数据将存储到 elrs_data_temp 缓冲区

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //初始化TIM4,用于0.1秒更新一次舵机状态
  HAL_TIM_Base_Start_IT(&htim4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Thruster_L(0);
  Thruster_R(0);
  while(elrs_data.Left_X==0)
  {
	     ELRS_Init();
  }
 //——————————————————————————————————————————————————————————————————————————————————————————————————————————//
  while (1)
   {
	  //初始化 huart2 的 DMA 接收,接收的数据将存储到 elrs_data_temp 缓冲区(正常来讲初始化不应该在循环里的，但是有问题，直接粗暴破解了)
	  //这里每周期接受一次数据
	     ELRS_Init();
	     Yaw_LED();
	     Yaw_Thruster();


	     //2024/12/8将舵机控制函数从TIM4中剥离，但是舵机开始抖动，如果抖动过于明显,后面考虑使用RTOS(话是这么说，让我先学()
         //右摇杆前后控制俯仰
	     AngleData=elrs_data.Right_Y;//这里不这么写后面不知道为什么左舵机会读成Left_Y(问号)(沟槽的奇妙小bug¿)
         SevorAngleSet_L(90+(AngleData/100*90));
         SevorAngleSet_R(90+(AngleData/100*90));



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
//——————————————————————————————————————————————————————————————————————————————————————————————————————————//
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
