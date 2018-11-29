#include "PowerModule.h"
#include "string.h"


#define LOCAL_CAN_ID                    (0x07)
#define DEVICE_ID                       (LOCAL_CAN_ID)

__IO uint16_t AdcAverage[4];
uint16_t TargetOutputVoltage=0;
uint16_t CurInputVoltage=0;
uint16_t CurOutputVoltage=0;
uint16_t CurOutputCurrent=0;
int16_t CurModuleTemperature=0;
char InputOverVoltageFlag=0;
char InputUnderVoltageFlag=0;
char OutputOverVoltageFlag=0;
char OutputOverCurrentFlag=0;
char ShortCircuitFlag=0;
char OverTemperatureFlag=0;
char FaultSendStopFlag=0;
char PowerStatusFlag=0;
uint32_t DacOutputValue=0;
uint32_t PowerOnDelayCounter=0;
uint32_t ShortCircuitRecoveryDelayCounter=0;
char MaybeShortCircuitFlag=0;
uint32_t ShortCircuitCheckDelayCounter=0;
CanRxMsgTypeDef        ValidRxMessage={0,0};

//OUTPUT
void FaultLightControl(GPIO_PinState lightState)
{
  HAL_GPIO_WritePin(FAULT_LIGHT_GPIO_Port, FAULT_LIGHT_Pin, lightState);
}

GPIO_PinState GetFaultLightStatus(void)
{
	HAL_GPIO_ReadPin(FAULT_LIGHT_GPIO_Port, FAULT_LIGHT_Pin);
}

void WorkLightControl(GPIO_PinState lightState)
{
	
  HAL_GPIO_WritePin(WORK_LIGHT_GPIO_Port, WORK_LIGHT_Pin, lightState);
}
void PowerControl(GPIO_PinState PowerState)
{
    HAL_GPIO_WritePin(POWER_CONTROL_GPIO_Port, POWER_CONTROL_Pin, PowerState);
}
GPIO_PinState GetPowerOutStatus(void)
{
	HAL_GPIO_ReadPin(POWER_CONTROL_GPIO_Port, POWER_CONTROL_Pin);
}

//INPUT
char  GetPowerStatus(void)
{
  return HAL_GPIO_ReadPin(POWER_STATE_SIGNAL_GPIO_Port, POWER_STATE_SIGNAL_Pin);
}
//ADC
/**
  * @brief  Computation of voltage (unit: mV) from ADC measurement digital
  *         value on range 12 bits.
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param ADC_DATA: Digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_DIGITAL_12BITS_TO_INPUT_VOLTAGE(ADC_DATA)                        \
(((uint16_t)((( (ADC_DATA) * VDD_APPLI / RANGE_12BITS *4.0f)+0.4f)*120.0f*10.0f))<600?0:((uint16_t)((( (ADC_DATA) * VDD_APPLI / RANGE_12BITS *4.0f)+0.4f)*120.0f*10.0f)))

#define COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_CURRENT(ADC_DATA)                        \
  ((uint16_t)( (ADC_DATA) * VDD_APPLI / RANGE_12BITS*1000.0f*0.02f*10.0f))

#define COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_VOLTAGE_SAMPLE(ADC_DATA)                        \
  ((uint16_t)( (ADC_DATA) * VDD_APPLI / RANGE_12BITS*46.8f/1.8f*10.0f))

void ADC_ExInit(ADC_HandleTypeDef* hadc)
{
  /* Run the ADC calibration */  
  if (HAL_ADCEx_Calibration_Start(hadc) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
  
  /*## Start ADC conversions #################################################*/
  
  /* Start ADC conversion on regular group with transfer by DMA */
  if (HAL_ADC_Start_DMA(hadc,
                        (uint32_t *)AdcAverage,
                        4
                       ) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
}
uint16_t GetOutputCurrent(void)
{
  return COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_CURRENT(AdcAverage[0]);
}
uint16_t GetInputVoltage(void)
{
  return COMPUTATION_DIGITAL_12BITS_TO_INPUT_VOLTAGE(AdcAverage[1]);
}

int16_t GetTemperature(void)
{
  return (-1786*(AdcAverage[2]*VDD_APPLI/RANGE_12BITS)+1821);
}
uint16_t GetOutputVoltage(void)
{
  return COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_VOLTAGE_SAMPLE(AdcAverage[3]);
}
//DAC

uint16_t OutputVoltageToDigital12Bits(float targetVoltage)
{
	float tempValue=0.0f;
  if(MAX_OUTPUT_V<targetVoltage)
  {
    targetVoltage=MAX_OUTPUT_V;
  }
  else if(MIN_OUTPUT_V>targetVoltage)
	{
		targetVoltage=MIN_OUTPUT_V;
	}
	tempValue=((targetVoltage*(-0.01146)+5.305)*RANGE_12BITS/VDD_APPLI);
	if(tempValue>RANGE_12BITS)
	{
		return RANGE_12BITS;
	}
	else if(tempValue<0)
	{
		return 0;
	}
	return tempValue;
}
//CAN
char ReceivedCanCommendFlag=0;

uint32_t GetLocalCanId(void)
{
	return LOCAL_CAN_ID;
}
uint32_t GetDeviceId(void)
{
	return DEVICE_ID;
}
void CAN_ExInit(CAN_HandleTypeDef* hcan)
{
  static CanTxMsgTypeDef        TxMessage;
  static CanRxMsgTypeDef        RxMessage;
  CAN_FilterConfTypeDef  sFilterConfig;
  /*## Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = (((uint32_t)HOST_CAN_ID<<21)&0xFFFF0000)>>16;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0xFFE0;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  /*## Configure Transmission process #####################################*/
  hcan->pTxMsg = &TxMessage;
  hcan->pRxMsg = &RxMessage;
  
  hcan->pTxMsg->StdId=LOCAL_CAN_ID;
  hcan->pTxMsg->ExtId = 0x00;
  hcan->pTxMsg->RTR = CAN_RTR_DATA;
  hcan->pTxMsg->IDE = CAN_ID_STD;
  hcan->pTxMsg->DLC = 8;
  
  /*## Start the Reception process and enable reception interrupt #########*/
  if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
}
void CAN_ReciveDataHandler(CAN_HandleTypeDef *hcan)
{
  if((POWER_DEVICE==((hcan->pRxMsg->Data[0]>>4)&0x0F))
    &&((DEVICE_ID==hcan->pRxMsg->Data[1])
    ||(BROADCAST_ID==hcan->pRxMsg->Data[1])))
  {
    if(RESET_FAULT_COMMAND==hcan->pRxMsg->Data[0])
    {
      FaultSendStopFlag=true;
    }
    else
    {
      ReceivedCanCommendFlag=true;
      memcpy(&ValidRxMessage,hcan->pRxMsg,sizeof(CanRxMsgTypeDef));
    }
  }
	/*## Start the Reception process and enable reception interrupt #########*/
	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
}

//Power Module Status
void GetPowerModuleStatus(void)
{
  CurInputVoltage=GetInputVoltage();
  CurOutputVoltage=GetOutputVoltage();
  CurOutputCurrent=GetOutputCurrent();
  CurModuleTemperature=GetTemperature();
  PowerStatusFlag=GetPowerStatus();
  DacOutputValue=OutputVoltageToDigital12Bits((ValidRxMessage.Data[2]<<8)|ValidRxMessage.Data[1]);
  if(CurInputVoltage>=IN_OVER_V_PROTECT_MIN)
  {
    InputOverVoltageFlag=true;
  }
  else if(CurInputVoltage<=IN_OVER_V_RECOVER_MAX)
  {
    InputOverVoltageFlag=false;
  }
  
  if((CurInputVoltage<=IN_UNDER_V_PROTECT_MIN)&&(PowerOnDelayCounter>=POWER_ON_MAX_DELAY))
  {
    InputUnderVoltageFlag=true;
  }
  else if(IN_UNDER_V_RECOVER_MIN<=CurInputVoltage)
  {
    InputUnderVoltageFlag=false;
  }
  
  if(CurOutputVoltage>=OUT_OVER_V_PROTECT_MIN)
  {
    OutputOverVoltageFlag=true;
  }
  
  if(CurOutputCurrent>=OUT_LIMIT_CURRENT_MIN)
  {
    OutputOverCurrentFlag=true;
  }
  else
  {
    OutputOverCurrentFlag=false;
  }
  
  if((CurModuleTemperature>=OVER_TEMPERATURE_VALUE)&&(PowerOnDelayCounter>=POWER_ON_MAX_DELAY))
  {
    OverTemperatureFlag=1;
  }
  else if(CurModuleTemperature<=OVER_TEMPERATURE_CANCEL_VALUE)
  {
    OverTemperatureFlag=0;
  }
  
  //短路判断
	if((POWER_ON==(GetPowerOutStatus()))&&(1==GetPowerStatus())&&(0==ShortCircuitFlag))
	{
		MaybeShortCircuitFlag=1;
		if((CurOutputVoltage<=50)&&(ShortCircuitCheckDelayCounter>500))//MAX_SHORT_CIRCUIT_CHECK_DELAY
		{
			ShortCircuitFlag=1;
		}
	}
	else if((1==ShortCircuitFlag)&&(SHORT_CIRCUIT_MAX_DELAY<=ShortCircuitRecoveryDelayCounter))
	{
		ShortCircuitFlag=0;
	}
	else if((POWER_ON==(GetPowerOutStatus()))&&(0==GetPowerStatus()))
	{
		MaybeShortCircuitFlag=0;
	}
}


//FLASH

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_127   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE   /* End @ of user Flash area */
/* Private variables ---------------------------------------------------------*/
uint32_t *const pPowerOnCounter=(uint32_t *)ADDR_FLASH_PAGE_127;
uint32_t *const pCanID=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*1));
uint32_t *const pInVolCalPara00=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*2));//Input Voltage Calibration Parameter 01;
uint32_t *const pInVolCalPara01=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*3));//Input Voltage Calibration Parameter 01;

uint32_t *const pOutVolCalPara00=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*4));//Output Voltage Calibration Parameter 01;
uint32_t *const pOutVolCalPara01=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*5));//Output Voltage Calibration Parameter 01;

uint32_t *const pOutCurCalPara00=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*6));//Output Current Calibration Parameter 01;
uint32_t *const pOutCurCalPara01=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*7));//Output Current Calibration Parameter 01;

uint32_t *const pTemCalPara00=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*8));//Temperature Calibration Parameter 01;
uint32_t *const pTemCalPara01=(uint32_t *)(ADDR_FLASH_PAGE_127+(4*9));//Temperature Calibration Parameter 01;


uint32_t PAGEError = 0;
__IO uint32_t MemoryProgramStatus = 0;
void WriteCanDataToFlash(CanRxMsgTypeDef *pCanRxMsg)
{
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
  if(CONFIG_COMMAND==pCanRxMsg->Data[0])
  {
    switch(pCanRxMsg->Data[1])
    {
      case 1:
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pCanID, pCanRxMsg->Data[2]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        break;
      case 2:
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pInVolCalPara00, (pCanRxMsg->Data[2]<<8)|pCanRxMsg->Data[3]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pInVolCalPara00, (pCanRxMsg->Data[4]<<8)|pCanRxMsg->Data[5]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        break;
      case 3:
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pOutVolCalPara00, (pCanRxMsg->Data[2]<<8)|pCanRxMsg->Data[3]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pOutVolCalPara00, (pCanRxMsg->Data[4]<<8)|pCanRxMsg->Data[5]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        break;
      case 4:
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pOutCurCalPara00, (pCanRxMsg->Data[2]<<8)|pCanRxMsg->Data[3]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pOutCurCalPara00, (pCanRxMsg->Data[4]<<8)|pCanRxMsg->Data[5]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        break;
      case 5:
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pTemCalPara00, (pCanRxMsg->Data[2]<<8)|pCanRxMsg->Data[3]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pTemCalPara00, (pCanRxMsg->Data[4]<<8)|pCanRxMsg->Data[5]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error */
          //返回FLASH错误信息？
        }
        break;
      default:
        break;
    }
  }
}




















