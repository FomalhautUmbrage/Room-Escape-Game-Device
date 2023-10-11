/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RedSelect 1
#define GreenSelect 2
#define BlueSelect 4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
   int mInputKeyRed=0;
   int mInputKeyGreen=0;
   int mInputKeyBlue=0;
   int mGameStart=0;
   int mGameLevel=0;
   int mInput=0;
   int mOutPut=0;
   int mRightRS=0;
   int mWin=7;
   int mInputConter=0;
   //int mFlashConter=0；
  int StepValue[10]={RedSelect,GreenSelect,BlueSelect,RedSelect,GreenSelect,BlueSelect,RedSelect,GreenSelect,BlueSelect,GreenSelect};
  int mFlashConter=0;
  int mflashStep=0;
  int mGameOver=BlueSelect;
  /* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GetInput(void)
	{
	if(HAL_GPIO_ReadPin(GPIOB, Button_Red_Pin))
		mInputKeyRed=0;//无输入
	else
		mInputKeyRed++;//有输入

	if(HAL_GPIO_ReadPin(GPIOB, Button_Green_Pin))
		mInputKeyGreen=0 ;//无输入
	else
		mInputKeyGreen++;//有输入
	mInput=mInput<<1;
	if(HAL_GPIO_ReadPin(GPIOB, Button_Blue_Pin))
		mInputKeyBlue=0 ;//无输入
	else
		mInputKeyBlue++;//有输入
	mInput=0;
	if(mInputKeyRed>5)
		mInput=1;
	mInput=mInput<<1;
	if(mInputKeyGreen>5)
		mInput++;
	mInput=mInput<<1;
	if(mInputKeyBlue>5)
		mInput++;
	if(mInput>0)
		mInputConter++;
	else
		mInputConter=0;
	if(mInputConter>1000)
		mInputConter=1000;
   // if(mInputConter==10)
	//mOutPut=mInput;
   // if(mInputConter==0)
    //	mOutPut=0;

	}
void OutPut(void)
	{
	int OutputTemp=mOutPut;
	if(OutputTemp%2)
		HAL_GPIO_WritePin(GPIOA, BlueLED_Pin, GPIO_PIN_SET);//输出高，点亮灯
	else
		HAL_GPIO_WritePin(GPIOA, BlueLED_Pin, GPIO_PIN_RESET);//输出高，灭灯
	OutputTemp=OutputTemp>>1;
	if(OutputTemp%2)
			HAL_GPIO_WritePin(GPIOA, GreenLED_Pin, GPIO_PIN_SET);//输出高，点亮灯
	else
			HAL_GPIO_WritePin(GPIOA, GreenLED_Pin, GPIO_PIN_RESET);//输出高，灭灯
	OutputTemp=OutputTemp>>1;
	if(OutputTemp%2)
		HAL_GPIO_WritePin(GPIOA, RedLED_Pin, GPIO_PIN_SET);//输出高，点亮灯
	else
	   HAL_GPIO_WritePin(GPIOA, RedLED_Pin, GPIO_PIN_RESET);//输出高，灭灯

	}
void Game(void)
	{
	if(mGameLevel==0)//限制空闲人时候可以开始
		if(mInput==(GreenSelect))//红的和绿的同时按下进入第一轮
		{
			if(mInputConter==30)
				mGameLevel=1;
		}

	if(mGameLevel==5)//胜利了，蓝按建复位
			if(mInput==(RedSelect))//红的和绿的同时按下进入第一轮
			{
				if(mInputConter==30)
					{
					mGameLevel=0;
					mOutPut=0;
					}
			}
	if(mGameLevel==6)//胜利了，蓝按建复位
				if(mInput==(RedSelect))//红的和绿的同时按下进入第一轮
				{
					if(mInputConter==30)
						{
						mGameLevel=0;
						mOutPut=0;
						}
				}
	if(mGameLevel==1)// 闪烁两次步进
		{
			mFlashConter++;//闪烁计数加

			if((mFlashConter/100)>=4)//闪烁两次
				{
					mFlashConter=0;
				    mflashStep++;
					if(mflashStep>9)//如果设定的流程走完返回重新循环
						mflashStep=0;
				}
			if(((mFlashConter/100)%2)==1)//100是闪烁时间,奇数的时候灭
					mOutPut=0;
			else
					mOutPut=StepValue[mflashStep];//输出当前步
			mRightRS=StepValue[mflashStep];//获得正确的结果
		}

	if(mGameLevel==2)// 闪烁3次步进
		{
			mFlashConter++;//闪烁计数加

			if((mFlashConter/100)>=6)//闪烁两次
				{
					mFlashConter=0;
				    mflashStep++;
					if(mflashStep>9)//如果设定的流程走完返回重新循环
						mflashStep=0;
				}
			if(((mFlashConter/100)%2)==1)//100是闪烁时间,奇数的时候灭
					mOutPut=0;
			else
					mOutPut=StepValue[mflashStep];//输出当前步
			mRightRS=StepValue[mflashStep];//获得正确的结果
		}

	if(mGameLevel==3)// 闪烁4次步进
		{
			mFlashConter++;//闪烁计数加

			if((mFlashConter/100)>=8)//闪烁两次
				{
					mFlashConter=0;
				    mflashStep++;
					if(mflashStep>9)//如果设定的流程走完返回重新循环
						mflashStep=0;
				}
			if(((mFlashConter/100)%2)==1)//100是闪烁时间,奇数的时候灭
					mOutPut=0;
			else
					mOutPut=StepValue[mflashStep];//输出当前步
			mRightRS=StepValue[mflashStep];//获得正确的结果
		}

	if(mGameLevel==4)// 闪烁5次步进
		{
			mFlashConter++;//闪烁计数加

			if((mFlashConter/100)>=10)//闪烁两次
				{
					mFlashConter=0;
				    mflashStep++;
					if(mflashStep>9)//如果设定的流程走完返回重新循环
						mflashStep=0;
				}
			if(((mFlashConter/100)%2)==1)//100是闪烁时间,奇数的时候灭
					mOutPut=0;
			else
					mOutPut=StepValue[mflashStep];//输出当前步
			mRightRS=StepValue[mflashStep];//获得正确的结果
		}
	if(mGameLevel==5)// 答错了锁定他
		{
			mOutPut=mGameOver;
		}
	if(mGameLevel==6)// 胜利了，所有灯亮起来
		{
			mOutPut=mWin;
		}
	//check 按键；
	if(mInputConter==10)
	if((mGameLevel<5)&(mGameLevel>0))
	{
		if(mInput==mRightRS)//判断按键是正确的。
			{
			 mRightRS=0;
			 mOutPut=0;
			 if(mGameLevel==4)
				 mGameLevel=6;//胜利了
			 else
				 mGameLevel++;//进入下一级
			}
		else//判断按键是错的。
			mGameLevel=5;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 //  HAL_GPIO_WritePin(GPIOA, RedLED_Pin|GreenLED_Pin|BlueLED_Pin, GPIO_PIN_RESET);
	  GetInput();
	  Game();
	  OutPut();
	  HAL_Delay(1);
     // HAL_GPIO_TogglePin(GPIOA, RedLED_Pin);
      //HAL_GPIO_ReadPin(GPIOB, Button_Green_Pin);
     // HAL_GPIO_WritePin(GPIOA, GreenLED_Pin, HAL_GPIO_ReadPin(GPIOB, Button_Green_Pin));
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
