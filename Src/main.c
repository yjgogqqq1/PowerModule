
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "PowerModule.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  ADC_ExInit(&hadc1);
  MX_CAN1_Init();
  CAN_ExInit(&hcan1);
  MX_DAC_Init();
	
  /* Infinite loop */
  while (1)
  {
    /* Start ADC conversion */
    /* Since sequencer is enabled in discontinuous mode, this will perform    */
    /* the conversion of the next rank in sequencer.                          */
    /* Note: For this example, conversion is triggered by software start,     */
    /*       therefore "HAL_ADC_Start()" must be called for each conversion.  */
    /*       Since DMA transfer has been initiated previously by function     */
    /*       "HAL_ADC_Start_DMA()", this function will keep DMA transfer      */
    /*       active.                                                          */
    
    
    
//	/* Wait for conversion completion before conditional check hereafter */
		HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    GetPowerModuleStatus();
		
    if((1==InputOverVoltageFlag)||(1==ShortCircuitFlag)
			||(1==InputUnderVoltageFlag)||(1==OutputOverVoltageFlag)
      ||(1==OverTemperatureFlag))
		{
      //处理
			PowerControl(POWER_OFF);
    }
    else if((POWER_ON_COMMAND==ValidRxMessage.Data[0]))
    {
      PowerControl(POWER_ON);
    }
    else if((POWER_OFF_COMMAND==ValidRxMessage.Data[0]))
    {
      //处理
			PowerControl(POWER_OFF);
    }
    

		
    if(1==OutputOverCurrentFlag)
    {
      //DAC降压输出由硬件实现，软件只置标志位
    }
    else
    {
      //DAC正常输出
      if((GetDeviceId()==ValidRxMessage.Data[1])&&(POWER_ON_COMMAND==ValidRxMessage.Data[0]))   //？？？？？需不需要添加故障标志位判断，有故障时不输出
      {
        /*##-3- Set DAC Channel1 DHR register ######################################*/
				if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, OutputVoltageToDigital12Bits((ValidRxMessage.Data[2]<<8)|ValidRxMessage.Data[3])) != HAL_OK)
				{
					/* Setting value Error */
					Error_Handler();
				}
				 /*##-4- Enable DAC Channel1 ################################################*/
				if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
				{
					/* Start Error */
					Error_Handler();
				}
      }
      else
      {
      } 
    }
		if(true==ReceivedCanCommendFlag)
    {
      ReceivedCanCommendFlag=false;
			if((QUERY_COMMAND==ValidRxMessage.Data[0])&&(GetDeviceId()==ValidRxMessage.Data[1]))
			{
				/* Set the data to be tranmitted */
				hcan1.pTxMsg->Data[0]=FEEDBACK_COMMAND;//CurOutputVoltage&0xff;//输出电压
				hcan1.pTxMsg->Data[1]=0x00;//CurOutputVoltage>>8;
				hcan1.pTxMsg->Data[2]=CurOutputVoltage>>8;//CurOutputCurrent&0xff;//输出电流
				hcan1.pTxMsg->Data[3]=CurOutputVoltage&0xff;//CurOutputCurrent>>8;
				hcan1.pTxMsg->Data[4]=0x00;
				hcan1.pTxMsg->Data[5]=0x00;
				hcan1.pTxMsg->Data[6]=0x00;
				/*##-- Start the Transmission process ###############################*/
				if (HAL_CAN_Transmit(&hcan1, 10) != HAL_OK)
				{
					/* Transmition Error */
					//Error_Handler();
				}
			}
    }
    
    
    //判断故障信息
    if((1==InputOverVoltageFlag)||(1==InputUnderVoltageFlag)||(1==OutputOverVoltageFlag)
      ||(1==OutputOverCurrentFlag)||(1==OverTemperatureFlag)||(1==ShortCircuitFlag))
    {
			FaultLightControl(LED_OFF);
      /* Set the data to be tranmitted */
      hcan1.pTxMsg->Data[0]=ERROR_COMMAND;//CurOutputVoltage&0xff;//输出电压
      hcan1.pTxMsg->Data[1]=0x00;//CurOutputVoltage>>8;
      hcan1.pTxMsg->Data[2]=0x00;//CurOutputCurrent&0xff;//输出电流
      hcan1.pTxMsg->Data[3]=0x00;//CurOutputCurrent>>8;
      hcan1.pTxMsg->Data[4]=0x00;
      hcan1.pTxMsg->Data[5]=0x00;
      //过温|输出短路|输出过压|输入欠压|输入过压|运行状态;//输出电流
      hcan1.pTxMsg->Data[4]=(OverTemperatureFlag<<5)|(ShortCircuitFlag<<4)
                        |(OutputOverVoltageFlag<<3)|(InputUnderVoltageFlag<<2)
                        |(InputOverVoltageFlag<<1)|(!GetPowerOutStatus());
      /*##-- Start the Transmission process ###############################*/
      if (HAL_CAN_Transmit(&hcan1, 10) != HAL_OK)
      {
        /* Transmition Error */
        //Error_Handler();
      }
			HAL_Delay(100);
    }
		else
		{
			FaultLightControl(LED_ON);
		}
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLLs ------------------------------------------------------*/
  /* PLL2 configuration: PLL2CLK = (HSE / HSEPrediv2Value) * PLL2MUL = (25 / 5) * 8 = 40 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLL2CLK / HSEPredivValue = 40 / 5 = 8 MHz */
  /* PLL configuration: PLLCLK = PREDIV1CLK * PLLMUL = 8 * 9 = 72 MHz */ 

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV5;
  oscinitstruct.Prediv1Source   = RCC_PREDIV1_SOURCE_PLL2;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  oscinitstruct.PLL2.PLL2State  = RCC_PLL2_ON;
  oscinitstruct.PLL2.PLL2MUL    = RCC_PLL2_MUL8;
  oscinitstruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
	/**Configure the Systick interrupt time 
	*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick 
	*/
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/**Configure the Systick interrupt time 
	*/
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
//void SystemClock_Config(void)
//{

//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//  RCC_PeriphCLKInitTypeDef PeriphClkInit;

//    /**Initializes the CPU, AHB and APB busses clocks 
//    */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
//  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
//  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Initializes the CPU, AHB and APB busses clocks 
//    */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
//  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Configure the Systick interrupt time 
//    */
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

//    /**Configure the Systick 
//    */
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//    /**Configure the Systick interrupt time 
//    */
//  __HAL_RCC_PLLI2S_ENABLE();

//  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
