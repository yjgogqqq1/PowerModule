
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
#include "i2c.h"
/* USER CODE BEGIN Includes */
#include "PowerModule.h"
#include<string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
unsigned int ErrorOccurTick=0;
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
  #ifdef USE_EEPROM
    MX_I2C1_Init();
  #endif
  MX_ADC1_Init();
  ADC_ExInit(&hadc1);
  MX_CAN1_Init();
  CAN_ExInit(&hcan1);
  #ifdef USE_EEPROM
    PowerOnCounterInc();
	  ReadCriticalDataFromEeprom(&hi2c1);
  #endif
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
		
    if((1==InputOverVoltageFlag)||(1==InputUnderVoltageFlag)
			||(1==OutputOverVolFlag[1])||(1==OutputOverCurFlag[1])
			||(1==OutputOverVolFlag[2])||(1==OutputOverCurFlag[2])
			||(1==OutputOverVolFlag[3])||(1==OutputOverCurFlag[3])
			||(1==OutputOverVolFlag[4])||(1==OutputOverCurFlag[4])
			||(1==OverTemperatureFlag))
    {
			if((false==FaultSendStopFlag)&&((0==ErrorOccurTick)||(0==(HAL_GetTick()-ErrorOccurTick)%30000)))
      {
				ErrorOccurTick=HAL_GetTick();
				/* Set the data to be tranmitted */
				hcan1.pTxMsg->Data[0]=ERROR_COMMAND;//CurOutputVoltage&0xff;//输出电压
				hcan1.pTxMsg->Data[1]=GetDeviceId();//CurOutputVoltage>>8;
				hcan1.pTxMsg->Data[2]=0x00;//CurOutputCurrent&0xff;//输出电流
				hcan1.pTxMsg->Data[3]=0x00;//CurOutputCurrent>>8;
				hcan1.pTxMsg->Data[4]=0x00;
				hcan1.pTxMsg->Data[5]=0x00;
				//过温|输出短路|输出过压|输入欠压|输入过压|运行状态;//输出电流
				hcan1.pTxMsg->Data[4]=(OverTemperatureFlag<<5)
													|(InputUnderVoltageFlag<<2)
													|(InputOverVoltageFlag<<1)|(!GetPowerOutStatus());
				//处理
				if((1==InputOverVoltageFlag)||(1==InputUnderVoltageFlag)||(1==OverTemperatureFlag))
				{
					PowerControl(0xFF,POWER_OFF);
					hcan1.pTxMsg->Data[7]=0;
					/*##-- Start the Transmission process ###############################*/
					if (HAL_CAN_Transmit(&hcan1, 10) != HAL_OK)
					{
					}
				}
				else
				{
					for(int i=1;i<5;i++)
					{
						if((1==OutputOverVolFlag[i])||(1==OutputOverCurFlag[i]))
						{
							hcan1.pTxMsg->Data[4]|=(OutputOverCurFlag[i]<<4)|(OutputOverVolFlag[i]<<3);
							hcan1.pTxMsg->Data[7]=i;
							/*##-- Start the Transmission process ###############################*/
							HAL_CAN_Transmit(&hcan1, 10);
						}
					}
				}
				FaultLightControl(LED_OFF);
      }
      #ifdef USE_EEPROM
        static uint32_t preErrorCode=0;
        //如果错误状态未发生改变，则不记录，如果发生改变则记录错误状态及程序运行时间
        if(preErrorCode!=hcan1.pTxMsg->Data[4])
        {
          NewErrorInforSave(hcan1.pTxMsg->Data[4],GetProgramRunMinutes());
          preErrorCode=hcan1.pTxMsg->Data[4];
        }
      #endif
    }
    else 
    {
      //错误恢复，发送错误状态清零指令
			ErrorOccurTick=0;
      if(LED_OFF==GetFaultLightStatus())
      {
        /* Set the data to be tranmitted */
				hcan1.pTxMsg->Data[0]=ERROR_COMMAND;//CurOutputVoltage&0xff;//输出电压
				hcan1.pTxMsg->Data[1]=GetDeviceId();//CurOutputVoltage>>8;
				hcan1.pTxMsg->Data[2]=0x00;//CurOutputCurrent&0xff;//输出电流
				hcan1.pTxMsg->Data[3]=0x00;//CurOutputCurrent>>8;
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
      //为下次出错发送指令做准备
      FaultSendStopFlag=false;
      //错误解决，打开指示灯，标识设备工作；
      FaultLightControl(LED_ON);
      
      if((POWER_ON_COMMAND==ValidRxMessage.Data[0]))
      {
        PowerControl(ValidRxMessage.Data[7],POWER_ON);
      }
      else if((POWER_OFF_COMMAND==ValidRxMessage.Data[0]))
      {
        PowerControl(ValidRxMessage.Data[7],POWER_OFF);
      }
    }
		if(true==ReceivedCanCommendFlag)
    {
      ReceivedCanCommendFlag=false;
			if((QUERY_COMMAND==ValidRxMessage.Data[0]))
			{
				/* Set the data to be tranmitted */
				hcan1.pTxMsg->Data[0]=FEEDBACK_COMMAND;//CurOutputVoltage&0xff;//输出电压
				hcan1.pTxMsg->Data[1]=GetDeviceId();//CurOutputVoltage>>8;
				hcan1.pTxMsg->Data[2]=CurOutputVoltage[ValidRxMessage.Data[0]]>>8;//CurOutputCurrent&0xff;//输出电流
				hcan1.pTxMsg->Data[3]=CurOutputVoltage[ValidRxMessage.Data[0]]&0xff;//CurOutputCurrent>>8;
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
		#ifdef USE_EEPROM
      if(true==GetReceivedDebugCommandFlag())
      {
        SetReceivedDebugCommandFlag(false);
        WriteDataToEeprom(pHi2c,hcan->pRxMsg);
      }
      if(true==GetReadErrorInforEnableFlag())
      {
        SetReadErrorInforEnableFlag(false);
        memcpy(hcan1.pTxMsg->Data,(const void *)(GetErrorInforListBaseAddr()+hcan1.pRxMsg->Data[2]),8);
        /*##-- Start the Transmission process ###############################*/
        if (HAL_CAN_Transmit(&hcan1, 10) != HAL_OK)
        {
          /* Transmition Error */
          //Error_Handler();
        }
      }
    #endif
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
