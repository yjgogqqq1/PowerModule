#include "PowerModule.h"
#include "string.h"

typedef struct
{
  uint32_t ErrorCode;
  uint32_t RunTotalTime;
} ErrorInfor;

ErrorInfor ErrorInforList[6];

#define LOCAL_DEFAULT_ID                    (0x03)
I2C_HandleTypeDef *pHi2c=NULL;
unsigned int LocalId=LOCAL_DEFAULT_ID;
void SetLocalId(unsigned int localId)
{
  LocalId=localId;
}
unsigned int GetLocalId()
{
  return LocalId;
}                
uint16_t InVolCalPara00           =0x00;
uint16_t InVolCalPara01           =0x00;
    
uint16_t OutVolSampleCalPara00    =0x00;
uint16_t OutVolSampleCalPara01    =0x00;
                                      
uint16_t OutCurCalPara00          =0x00;
uint16_t OutCurCalPara01          =0x00;
                                      
uint16_t TemCalPara00             =0x00;
uint16_t TemCalPara01             =0x00;
                                      
uint16_t OutVolCalPara00          =0x00;
uint16_t OutVolCalPara01          =0x00;

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
void FaultLightControl(LightStatus lightState)
{
  HAL_GPIO_WritePin(FAULT_LIGHT_GPIO_Port, FAULT_LIGHT_Pin, (GPIO_PinState)lightState);
}

GPIO_PinState GetFaultLightStatus(void)
{
	return HAL_GPIO_ReadPin(FAULT_LIGHT_GPIO_Port, FAULT_LIGHT_Pin);
}

void WorkLightControl(GPIO_PinState lightState)
{
	
  HAL_GPIO_WritePin(WORK_LIGHT_GPIO_Port, WORK_LIGHT_Pin, lightState);
}
void PowerControl(PowerStatus PowerState)
{
    HAL_GPIO_WritePin(POWER_CONTROL_GPIO_Port, POWER_CONTROL_Pin, (GPIO_PinState)PowerState);
}
GPIO_PinState GetPowerOutStatus(void)
{
	return HAL_GPIO_ReadPin(POWER_CONTROL_GPIO_Port, POWER_CONTROL_Pin);
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
	return LOCAL_DEFAULT_ID;
}
uint32_t GetDeviceId(void)
{
	return LOCAL_DEFAULT_ID;
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
  
  hcan->pTxMsg->StdId=LOCAL_DEFAULT_ID;
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
  if((CONFIG_COMMAND==hcan->pRxMsg->Data[0])
    &&(false==GetBanFlashOperateFlag()))
  {
    WriteDataToEeprom(pHi2c,hcan->pRxMsg);
  }
  else if((POWER_DEVICE==((hcan->pRxMsg->Data[0]>>4)&0x0F))
    &&((LocalId==hcan->pRxMsg->Data[1])
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
/* Base address of the Flash sectors */
#define ADDR_FLASH_PAGE_127   ((uint32_t)0x0801FC00) /* Base @ of Page 127, 1 Kbytes */

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_127   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE   /* End @ of user Flash area */
/* Private variables ---------------------------------------------------------*/
uint32_t *pPowerOnCounter=(uint32_t *)ADDR_FLASH_PAGE_127;
unsigned char BanFlashOperateFlag=false;
unsigned char GetBanFlashOperateFlag(void)
{
  return BanFlashOperateFlag;
}
uint32_t PAGEError = 0;
__IO uint32_t MemoryProgramStatus = 0;
/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;
void PowerOnCounterInc(void)
{
	unsigned int powerOnCounter=*pPowerOnCounter;
	/*Variable used for Erase procedure*/
	FLASH_EraseInitTypeDef EraseInitStruct;

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
	EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
	
	if(3>*pPowerOnCounter)
  {
		/* Unlock the Flash to enable the flash control register access *************/
		HAL_FLASH_Unlock();
		/* Fill EraseInit structure*/
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
		{
			/* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
			_Error_Handler(__FILE__,__LINE__);
		}
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pPowerOnCounter, ++powerOnCounter) != HAL_OK)
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
			_Error_Handler(__FILE__,__LINE__);
    }
    HAL_FLASH_Lock();
  }
  else if(3<*pPowerOnCounter)		//防止FLASH默认值干扰
	{
		/* Unlock the Flash to enable the flash control register access *************/
		HAL_FLASH_Unlock();
		
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
		{
			_Error_Handler(__FILE__,__LINE__);
		}
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)pPowerOnCounter, 1) != HAL_OK)
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
			_Error_Handler(__FILE__,__LINE__);
    }
    HAL_FLASH_Lock();
	}
	else
  {
    BanFlashOperateFlag=true;
  }
}
//eeprom
/* 
 * EEPROM 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */
/* EEPROM Addresses defines */ 
#define EEPROM_I2C_W_ADDRESS                         (0xA0)
#define EEPROM_I2C_R_ADDRESS                         (0xA1)

#define EEPROM_BASE_ADDRESS                 (0x0000)
uint16_t LocalID_Addr                 =EEPROM_BASE_ADDRESS;
uint16_t InVolCalPara00_Addr          =EEPROM_BASE_ADDRESS+(4*1);//Input Voltage Calibration Parameter 01;
uint16_t InVolCalPara01_Addr          =EEPROM_BASE_ADDRESS+(4*2);//Input Voltage Calibration Parameter 01;
    
uint16_t OutVolSampleCalPara00_Addr   =EEPROM_BASE_ADDRESS+(4*3);//Output Voltage Sample Calibration Parameter 01;
uint16_t OutVolSampleCalPara01_Addr   =EEPROM_BASE_ADDRESS+(4*4);//Output Voltage Sample Calibration Parameter 01;
                                                            
uint16_t OutCurCalPara00_Addr         =EEPROM_BASE_ADDRESS+(4*5);//Output Current Calibration Parameter 01;
uint16_t OutCurCalPara01_Addr         =EEPROM_BASE_ADDRESS+(4*6);//Output Current Calibration Parameter 01;
                                                            
uint16_t TemCalPara00_Addr            =EEPROM_BASE_ADDRESS+(4*7);//Temperature Calibration Parameter 01;
uint16_t TemCalPara01_Addr            =EEPROM_BASE_ADDRESS+(4*8);//Temperature Calibration Parameter 01;
                                                            
uint16_t OutVolCalPara00_Addr         =EEPROM_BASE_ADDRESS+(4*9);//Output Voltage Calibration Parameter 01;
uint16_t OutVolCalPara01_Addr         =EEPROM_BASE_ADDRESS+(4*10);//Output Voltage Calibration Parameter 01;
uint16_t ErrorInforList_Addr          =EEPROM_BASE_ADDRESS+(4*16);
HAL_StatusTypeDef EepromWrite(I2C_HandleTypeDef *hi2c,uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
  return HAL_I2C_Mem_Write(hi2c,EEPROM_I2C_W_ADDRESS,MemAddress,I2C_MEMADD_SIZE_8BIT,pData,Size,2000);
}

HAL_StatusTypeDef EepromRead(I2C_HandleTypeDef *hi2c,uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
  return HAL_I2C_Mem_Read(hi2c,EEPROM_I2C_R_ADDRESS,MemAddress,I2C_MEMADD_SIZE_16BIT,pData,Size,2000);
}

void WriteDataToEeprom(I2C_HandleTypeDef *hi2c,CanRxMsgTypeDef *pCanRxMsg)
{
  switch(pCanRxMsg->Data[1])
  {
    case 1:       //读取最近几次故障
      break;
    case 2:       //清空最近几次故障
      break;
    case 3:       //读取最后一次出错时程序运行时间
      break;
    case 4:       //读取最后一次出错时程序运行时间
      break;
    case 5:       //读取本机ID
      
      break;
    case 6:       //设置本机ID
      LocalId=pCanRxMsg->Data[2];
      if (EepromWrite(hi2c,LocalID_Addr,(uint8_t *)&LocalId,sizeof(LocalId)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      break;
    case 7:       //设置输入电压采集参数
      InVolCalPara00=(pCanRxMsg->Data[2]<<8) | pCanRxMsg->Data[3];
      if (EepromWrite(hi2c,InVolCalPara00_Addr,(uint8_t *)&InVolCalPara00,sizeof(InVolCalPara00))!= HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      InVolCalPara01=(pCanRxMsg->Data[4]<<8) | pCanRxMsg->Data[5];
      if (EepromWrite(hi2c,InVolCalPara01_Addr,(uint8_t *)&InVolCalPara01,sizeof(InVolCalPara01)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      break;
    case 8:       //设置输出电压采集参数
      OutVolSampleCalPara00=(pCanRxMsg->Data[2]<<8) | pCanRxMsg->Data[3];
      if (EepromWrite(hi2c,OutVolSampleCalPara00_Addr,(uint8_t *)&OutVolSampleCalPara00,sizeof(OutVolSampleCalPara00)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      OutVolSampleCalPara01=(pCanRxMsg->Data[4]<<8) | pCanRxMsg->Data[5];
      if (EepromWrite(hi2c,OutVolSampleCalPara01_Addr,(uint8_t *)&OutVolSampleCalPara01,sizeof(OutVolSampleCalPara01)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      break;
    case 9:       //设置输出电流采集参数
      OutCurCalPara00=(pCanRxMsg->Data[2]<<8) | pCanRxMsg->Data[3];
      if (EepromWrite(hi2c,OutCurCalPara00_Addr,(uint8_t *)&OutCurCalPara00,sizeof(OutCurCalPara00)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      OutCurCalPara01=(pCanRxMsg->Data[4]<<8) | pCanRxMsg->Data[5];
      if (EepromWrite(hi2c,OutCurCalPara01_Addr,(uint8_t *)&OutCurCalPara01,sizeof(OutCurCalPara01)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      break;
    case 10:       //设置温度采集参数
      TemCalPara00=(pCanRxMsg->Data[2]<<8) | pCanRxMsg->Data[3];
      if (EepromWrite(hi2c,TemCalPara00_Addr,(uint8_t *)&TemCalPara00,sizeof(TemCalPara00)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      TemCalPara01=(pCanRxMsg->Data[4]<<8) | pCanRxMsg->Data[5];
      if (EepromWrite(hi2c,TemCalPara01_Addr,(uint8_t *)&TemCalPara01,sizeof(TemCalPara01)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      break;
    case 11:      //设置输出电压调节参数
      OutVolCalPara00=(pCanRxMsg->Data[2]<<8) | pCanRxMsg->Data[3];
      if (EepromWrite(hi2c,OutVolCalPara00_Addr,(uint8_t *)&OutVolCalPara00,sizeof(OutVolCalPara00)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
        //返回FLASH错误信息？
      }
      OutVolCalPara01=(pCanRxMsg->Data[4]<<8) | pCanRxMsg->Data[5];
      if (EepromWrite(hi2c,OutVolCalPara01_Addr,(uint8_t *)&OutVolCalPara01,sizeof(OutVolCalPara00)) != HAL_OK)
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
void ReadCriticalDataFromEeprom(I2C_HandleTypeDef *hi2c)
{
  pHi2c=hi2c;
  //读取本地ID
  while (EepromRead(hi2c,LocalID_Addr,(uint8_t *)&LocalId,sizeof(LocalId)) != HAL_OK)
  {
		//while(1);
  }
  
  //读取输入电压采集参数
  if (EepromRead(hi2c,InVolCalPara00_Addr,(uint8_t *)&InVolCalPara00,sizeof(InVolCalPara00))!= HAL_OK)
  {
		while(1);
  }
  if (EepromWrite(hi2c,InVolCalPara01_Addr,(uint8_t *)&InVolCalPara01,sizeof(InVolCalPara01)) != HAL_OK)
  {
  }

  //读取输出电压采集参数
  if (EepromRead(hi2c,OutVolSampleCalPara00_Addr,(uint8_t *)&OutVolSampleCalPara00,sizeof(OutVolSampleCalPara00)) != HAL_OK)
  {
		while(1);
  }
  if (EepromRead(hi2c,OutVolSampleCalPara01_Addr,(uint8_t *)&OutVolSampleCalPara01,sizeof(OutVolSampleCalPara01)) != HAL_OK)
  {
  }
  
  //读取输出电流采集参数
  if (EepromRead(hi2c,OutCurCalPara00_Addr,(uint8_t *)&OutCurCalPara00,sizeof(OutCurCalPara00)) != HAL_OK)
  {
		while(1);
  }

  if (EepromRead(hi2c,OutCurCalPara01_Addr,(uint8_t *)&OutCurCalPara01,sizeof(OutCurCalPara01)) != HAL_OK)
  {
		while(1);
  }
  
  //读取温度采集参数
  if (EepromRead(hi2c,TemCalPara00_Addr,(uint8_t *)&TemCalPara00,sizeof(TemCalPara00)) != HAL_OK)
  {
		while(1);
  }
  if (EepromRead(hi2c,TemCalPara01_Addr,(uint8_t *)&TemCalPara01,sizeof(TemCalPara01)) != HAL_OK)
  {
		while(1);
  }
  
  //读取输出电压调节参数
  if (EepromRead(hi2c,OutVolCalPara00_Addr,(uint8_t *)&OutVolCalPara00,sizeof(OutVolCalPara00)) != HAL_OK)
  {
		while(1);
  }
  if (EepromRead(hi2c,OutVolCalPara01_Addr,(uint8_t *)&OutVolCalPara01,sizeof(OutVolCalPara00)) != HAL_OK)
  {
  }
  
  //读取近几次错误信息列表
  if (EepromRead(hi2c,ErrorInforList_Addr,(uint8_t *)&ErrorInforList,sizeof(ErrorInforList)) != HAL_OK)
  {
		while(1);
  }
}

void NewErrorInforSave(unsigned char errorCode,unsigned int runTotalTime)
{
  for(int i=7;i>0;i--)
  {
    ErrorInforList[i]=ErrorInforList[i-1];
  }
  ErrorInforList[0].ErrorCode=errorCode;
  ErrorInforList[0].RunTotalTime=runTotalTime;
  EepromWrite(pHi2c,ErrorInforList_Addr,(uint8_t *)ErrorInforList,sizeof(ErrorInforList));
}
//程序运行时间计时
#define TIME_BASE       (1*60*1000)       //1分钟
unsigned int ProgramRunMinutes=0;
unsigned int GetProgramRunMinutes(void)
{
  return ProgramRunMinutes;
}
void ProgramRunTiming(void)
{
  static unsigned short timeCounter=0;
  timeCounter++;
  if(timeCounter>=TIME_BASE)
  {
    ProgramRunMinutes++;
  }
}













