#include "PowerModule.h"
#include "string.h"


ErrorInfor ErrorInforList[10];
ErrorInfor *GetErrorInforListBaseAddr(void)
{
	return ErrorInforList;
}
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
// #define IP_VOL_PARA_KA        ()
// #define IP_VOL_PARA_KB        ()
short InVolCalParaA           =4800;
short InVolCalParaB           =480;
// #define OP_VOL_SAMPLE_PARA_KA ()    
// #define OP_VOL_SAMPLE_PARA_KB ()
short OutVolSampleCalParaA    =260;
short OutVolSampleCalParaB    =0;
// #define OP_CUR_PARA_KA        () 
// #define OP_CUR_PARA_KB        ()                                    
short OutCurCalParaA          =200;
short OutCurCalParaB          =0;
// #define TEM_PARA_KA           ()   
// #define TEM_PARA_KB           ()                                     
short TemCalParaA             =-1786;
short TemCalParaB             =1821;
#define OP_VOL_PARA_KA       (10000.0f)   
#define OP_VOL_PARA_KB       (1.0f)                                  
short OutVolCalParaA          =10000;
short OutVolCalParaB          =0;

__IO unsigned short AdcAverage[4];
#define OP_CUR_SAMPLING_INDEX       (0)
#define IP_VOL_SAMPLING_INDEX       (1)
#define TEMPERATURE_SAMPLING_INDEX  (2)
#define OP_VOL_SAMPLING_INDEX       (3)
unsigned short TargetOutputVoltage=0;
unsigned short CurInputVoltage=0;
unsigned short CurOutputVoltage=0;
unsigned short CurOutputCurrent=0;
short CurModuleTemperature=0;
char InputOverVoltageFlag=0;
char InputUnderVoltageFlag=0;
char OutputOverVoltageFlag=0;
char OutputOverCurrentFlag=0;
char ShortCircuitFlag=0;
char OverTemperatureFlag=0;
char FaultSendStopFlag=0;
char PowerStatusFlag=0;
//DAC输出12位值
static unsigned short DacOutputValue=0;
void SetDacOutputValue(unsigned short dacOutputValue){ DacOutputValue=dacOutputValue;}
unsigned short GetDacOutputValue(){ return DacOutputValue;}

unsigned int PowerOnDelayCounter=0;
unsigned int ShortCircuitRecoveryDelayCounter=0;
char MaybeShortCircuitFlag=0;
unsigned int ShortCircuitCheckDelayCounter=0;
CanRxMsgTypeDef        ValidRxMessage={0,0};
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
unsigned short LocalID_Addr                 =EEPROM_BASE_ADDRESS;
unsigned short InVolCalParaA_Addr          =EEPROM_BASE_ADDRESS+(4*1);//Input Voltage Calibration Parameter 01;
unsigned short InVolCalParaB_Addr          =EEPROM_BASE_ADDRESS+(4*2);//Input Voltage Calibration Parameter 01;
    
unsigned short OutVolSampleCalParaA_Addr   =EEPROM_BASE_ADDRESS+(4*3);//Output Voltage Sample Calibration Parameter 01;
unsigned short OutVolSampleCalParaB_Addr   =EEPROM_BASE_ADDRESS+(4*4);//Output Voltage Sample Calibration Parameter 01;
                                                            
unsigned short OutCurCalParaA_Addr         =EEPROM_BASE_ADDRESS+(4*5);//Output Current Calibration Parameter 01;
unsigned short OutCurCalParaB_Addr         =EEPROM_BASE_ADDRESS+(4*6);//Output Current Calibration Parameter 01;
                                                            
unsigned short TemCalParaA_Addr            =EEPROM_BASE_ADDRESS+(4*7);//Temperature Calibration Parameter 01;
unsigned short TemCalParaB_Addr            =EEPROM_BASE_ADDRESS+(4*8);//Temperature Calibration Parameter 01;
                                                            
unsigned short OutVolCalParaA_Addr         =EEPROM_BASE_ADDRESS+(4*9);//Output Voltage Calibration Parameter 01;
unsigned short OutVolCalParaB_Addr         =EEPROM_BASE_ADDRESS+(4*10);//Output Voltage Calibration Parameter 01;
unsigned short ErrorInforList_Addr         =EEPROM_BASE_ADDRESS+(4*16);
HAL_StatusTypeDef EepromWrite(I2C_HandleTypeDef *hi2c,unsigned short MemAddress, unsigned char *pData, unsigned short Size)
{
  HAL_Delay(5);
  return HAL_I2C_Mem_Write(hi2c,EEPROM_I2C_W_ADDRESS,MemAddress,I2C_MEMADD_SIZE_8BIT,pData,Size,2000);
}

HAL_StatusTypeDef EepromRead(I2C_HandleTypeDef *hi2c,unsigned short MemAddress, unsigned char *pData, unsigned short Size)
{
  HAL_Delay(5);
  return HAL_I2C_Mem_Read(hi2c,EEPROM_I2C_R_ADDRESS,MemAddress,I2C_MEMADD_SIZE_8BIT,pData,Size,2000);
}
char ReadErrorInforEnableFlag=0;
void SetReadErrorInforEnableFlag(char readErrorInforEnableFlag)
{
	ReadErrorInforEnableFlag=readErrorInforEnableFlag;
}
char GetReadErrorInforEnableFlag()
{
	return ReadErrorInforEnableFlag;
}
void DebugConmmandProcess(I2C_HandleTypeDef *hi2c,CAN_HandleTypeDef *pHcan)
{
  switch(pHcan->pRxMsg->Data[1])
  {
    case 1:       //读取最近几次故障
			SetReadErrorInforEnableFlag(1);
      break;
    case 5:       //读取本机ID
      break;
    case 6:       //设置本机ID
      LocalId=pHcan->pRxMsg->Data[2];
      if (EepromWrite(hi2c,LocalID_Addr,(unsigned char *)&LocalId,sizeof(LocalId)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      break;
    case 7:       //设置输入电压采集参数
      InVolCalParaA=(pHcan->pRxMsg->Data[2]<<8) | pHcan->pRxMsg->Data[3];
      if (EepromWrite(hi2c,InVolCalParaA_Addr,(unsigned char *)&InVolCalParaA,sizeof(InVolCalParaA))!= HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      InVolCalParaB=(pHcan->pRxMsg->Data[4]<<8) | pHcan->pRxMsg->Data[5];
      if (EepromWrite(hi2c,InVolCalParaB_Addr,(unsigned char *)&InVolCalParaB,sizeof(InVolCalParaB)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      break;
    case 8:       //设置输出电压采集参数
      OutVolSampleCalParaA=(pHcan->pRxMsg->Data[2]<<8) | pHcan->pRxMsg->Data[3];
      if (EepromWrite(hi2c,OutVolSampleCalParaA_Addr,(unsigned char *)&OutVolSampleCalParaA,sizeof(OutVolSampleCalParaA)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      OutVolSampleCalParaB=(pHcan->pRxMsg->Data[4]<<8) | pHcan->pRxMsg->Data[5];
      if (EepromWrite(hi2c,OutVolSampleCalParaB_Addr,(unsigned char *)&OutVolSampleCalParaB,sizeof(OutVolSampleCalParaB)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      break;
    case 9:       //设置输出电流采集参数
      OutCurCalParaA=(pHcan->pRxMsg->Data[2]<<8) | pHcan->pRxMsg->Data[3];
      if (EepromWrite(hi2c,OutCurCalParaA_Addr,(unsigned char *)&OutCurCalParaA,sizeof(OutCurCalParaA)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      OutCurCalParaB=(pHcan->pRxMsg->Data[4]<<8) | pHcan->pRxMsg->Data[5];
      if (EepromWrite(hi2c,OutCurCalParaB_Addr,(unsigned char *)&OutCurCalParaB,sizeof(OutCurCalParaB)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      break;
    case 10:       //设置温度采集参数
      TemCalParaA=(pHcan->pRxMsg->Data[2]<<8) | pHcan->pRxMsg->Data[3];
      if (EepromWrite(hi2c,TemCalParaA_Addr,(unsigned char *)&TemCalParaA,sizeof(TemCalParaA)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      TemCalParaB=(pHcan->pRxMsg->Data[4]<<8) | pHcan->pRxMsg->Data[5];
      if (EepromWrite(hi2c,TemCalParaB_Addr,(unsigned char *)&TemCalParaB,sizeof(TemCalParaB)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      break;
    case 11:      //设置输出电压调节参数
      OutVolCalParaA=(pHcan->pRxMsg->Data[2]<<8) | pHcan->pRxMsg->Data[3];
      if (EepromWrite(hi2c,OutVolCalParaA_Addr,(unsigned char *)&OutVolCalParaA,sizeof(OutVolCalParaA)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
      OutVolCalParaB=(pHcan->pRxMsg->Data[4]<<8) | pHcan->pRxMsg->Data[5];
      if (EepromWrite(hi2c,OutVolCalParaB_Addr,(unsigned char *)&OutVolCalParaB,sizeof(OutVolCalParaB)) != HAL_OK)
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error */
      }
    case 12:    //读取输入电压AD值
        pHcan->pTxMsg->Data[0]=AdcAverage[IP_VOL_SAMPLING_INDEX]>>8;
				pHcan->pTxMsg->Data[1]=AdcAverage[IP_VOL_SAMPLING_INDEX]&0xff;
				pHcan->pTxMsg->Data[2]=0x00;
				pHcan->pTxMsg->Data[3]=0x00;
				pHcan->pTxMsg->Data[4]=0x00;
				pHcan->pTxMsg->Data[5]=0x00;
				pHcan->pTxMsg->Data[6]=0x00;
				/*##-- Start the Transmission process ###############################*/
				HAL_CAN_Transmit(pHcan, 10);
      break;
    case 13:    //读取输出电压AD值
        pHcan->pTxMsg->Data[0]=AdcAverage[OP_VOL_SAMPLING_INDEX]>>8;
				pHcan->pTxMsg->Data[1]=AdcAverage[OP_VOL_SAMPLING_INDEX]&0xff;
				pHcan->pTxMsg->Data[2]=0x00;
				pHcan->pTxMsg->Data[3]=0x00;
				pHcan->pTxMsg->Data[5]=0x00;
				pHcan->pTxMsg->Data[6]=0x00;
				/*##-- Start the Transmission process ###############################*/
				HAL_CAN_Transmit(pHcan, 10);
      break;
    case 14:    //读取输出电流AD值
        pHcan->pTxMsg->Data[0]=AdcAverage[OP_CUR_SAMPLING_INDEX]>>8;
				pHcan->pTxMsg->Data[1]=AdcAverage[OP_CUR_SAMPLING_INDEX]&0xff;
				pHcan->pTxMsg->Data[2]=0x00;
				pHcan->pTxMsg->Data[3]=0x00;
				pHcan->pTxMsg->Data[4]=0x00;
				pHcan->pTxMsg->Data[5]=0x00;
				pHcan->pTxMsg->Data[6]=0x00;
				/*##-- Start the Transmission process ###############################*/
				HAL_CAN_Transmit(pHcan, 10);
      break;
    case 15:    //读取温度AD值
        pHcan->pTxMsg->Data[0]=AdcAverage[TEMPERATURE_SAMPLING_INDEX]>>8;
				pHcan->pTxMsg->Data[1]=AdcAverage[TEMPERATURE_SAMPLING_INDEX]&0xff;
				pHcan->pTxMsg->Data[2]=0x00;
				pHcan->pTxMsg->Data[3]=0x00;
				pHcan->pTxMsg->Data[4]=0x00;
				pHcan->pTxMsg->Data[5]=0x00;
				pHcan->pTxMsg->Data[6]=0x00;
				/*##-- Start the Transmission process ###############################*/
				HAL_CAN_Transmit(pHcan, 10);
      break;
    case 16:    //读取运算后的DA值
        pHcan->pTxMsg->Data[0]=GetDacOutputValue()>>8;
				pHcan->pTxMsg->Data[1]=GetDacOutputValue()&0xff;
				pHcan->pTxMsg->Data[2]=0x00;
				pHcan->pTxMsg->Data[3]=0x00;
				pHcan->pTxMsg->Data[4]=0x00;
				pHcan->pTxMsg->Data[5]=0x00;
				pHcan->pTxMsg->Data[6]=0x00;
				/*##-- Start the Transmission process ###############################*/
				HAL_CAN_Transmit(pHcan, 10);
      break;
    default:
      break;
  }
}
void ReadCriticalDataFromEeprom(I2C_HandleTypeDef *hi2c)
{
  unsigned int tU32=1;
  unsigned short t16=0;
  pHi2c=hi2c;
  //读取本地ID
  if((EepromRead(hi2c,LocalID_Addr,(unsigned char *)&tU32,sizeof(tU32))== HAL_OK)&&((0!=~tU32)))
  {
      LocalId=tU32;
  }
  
  //读取输入电压采集参数
  if ((EepromRead(hi2c,InVolCalParaA_Addr,(unsigned char *)&t16,sizeof(t16))== HAL_OK)&&(0!=~t16))
  {
    InVolCalParaA=t16;
  }
  if ((EepromWrite(hi2c,InVolCalParaB_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK)&&(0!=~t16))
  {
    InVolCalParaB=t16;
  }

  //读取输出电压采集参数
  if ((EepromRead(hi2c,OutVolSampleCalParaA_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK)&&(0!=~t16))
  {
		OutVolSampleCalParaA=t16;
  }
  if ((EepromRead(hi2c,OutVolSampleCalParaB_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK)&&(0!=~t16))
  {
    OutVolSampleCalParaB=t16;
  }
  
  //读取输出电流采集参数
  if (EepromRead(hi2c,OutCurCalParaA_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK&&(0!=~t16))
  {
		OutCurCalParaA=t16;
  }

  if (EepromRead(hi2c,OutCurCalParaB_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK&&(0!=~t16))
  {
		OutCurCalParaB=t16;
  }
  
  //读取温度采集参数
  if (EepromRead(hi2c,TemCalParaA_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK&&(0!=~t16))
  {
		TemCalParaA=t16;
  }
  if (EepromRead(hi2c,TemCalParaB_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK&&(0!=~t16))
  {
		TemCalParaB=t16;
  }
  
  //读取输出电压调节参数
  if (EepromRead(hi2c,OutVolCalParaA_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK&&(0!=~t16))
  {
		OutVolCalParaA=t16;
  }
  if (EepromRead(hi2c,OutVolCalParaB_Addr,(unsigned char *)&t16,sizeof(t16)) == HAL_OK&&(0!=~t16))
  {
    OutVolCalParaB=t16;
  }
  
  //读取近几次错误信息列表
	ErrorInfor errorInforList[10];
  for(int i=0;i<10;i++)
  {
    if (EepromRead(hi2c,ErrorInforList_Addr+i*8,(uint8_t *)&errorInforList[i],sizeof(errorInforList[i])) == HAL_OK)
    {
      if((0!=~errorInforList[i].ErrorCode)&&(0!=~errorInforList[i].RunTotalTime))
      {
        ErrorInforList[i]=errorInforList[i];
      }
    }
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
  for(int i=0;i<10;i++)
  {
    EepromWrite(pHi2c,ErrorInforList_Addr+i*8,(uint8_t *)&ErrorInforList[i],sizeof(ErrorInforList[i]));
  }
}
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
(((unsigned short)((( (ADC_DATA) * VDD_APPLI / RANGE_12BITS *4.0f)+0.4f)*120.0f*10.0f))<600?0:((unsigned short)((( (ADC_DATA) * VDD_APPLI / RANGE_12BITS *4.0f)+0.4f)*120.0f*10.0f)))

#define COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_CURRENT(ADC_DATA)                        \
  ((unsigned short)( (ADC_DATA) * VDD_APPLI / RANGE_12BITS*1000.0f*0.02f*10.0f))

#define COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_VOLTAGE_SAMPLE(ADC_DATA)                        \
  ((unsigned short)( (ADC_DATA) * VDD_APPLI / RANGE_12BITS*46.8f/1.8f*10.0f))
unsigned short ComputeDigital12BitsToInputVoltage(unsigned short adcData)
{
  #ifdef USE_EEPROM
  return (((unsigned short)(InVolCalParaA*adcData * VDD_APPLI / RANGE_12BITS+InVolCalParaB))<600)?0:((unsigned short)((InVolCalParaA*(adcData * VDD_APPLI / RANGE_12BITS)+InVolCalParaB)*10));
  #else
  return COMPUTATION_DIGITAL_12BITS_TO_INPUT_VOLTAGE(adcData);
  #endif
}
unsigned short ComputeDigital12bitsToOutputCurrent(unsigned short adcData)
{
  #ifdef USE_EEPROM
  return ((unsigned short)(OutCurCalParaA*adcData*VDD_APPLI/RANGE_12BITS+OutCurCalParaB));
  #else
  return COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_CURRENT(adcData);
  #endif
}

unsigned short ComputeDigital12bitsToOutputVoltageSample(unsigned short adcData)
{
  #ifdef USE_EEPROM
    return ((unsigned short)(OutVolSampleCalParaA*adcData * VDD_APPLI / RANGE_12BITS+OutVolSampleCalParaB));
  #else
    return COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_VOLTAGE_SAMPLE(adcData);
  #endif  
}
unsigned short ComputeDigital12BitsToTemperature(unsigned short adcData)
{
  #ifdef USE_EEPROM
    return (TemCalParaA*(adcData*VDD_APPLI/RANGE_12BITS)+TemCalParaB);    
  #else
    return (-1786*(adcData*VDD_APPLI/RANGE_12BITS)+1821);
  #endif
}
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
                        (unsigned int *)AdcAverage,
                        4
                       ) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
}
unsigned short GetOutputCurrent(void)
{
  return ComputeDigital12bitsToOutputCurrent(AdcAverage[OP_CUR_SAMPLING_INDEX]);
}
unsigned short GetInputVoltage(void)
{
  return ComputeDigital12BitsToInputVoltage(AdcAverage[IP_VOL_SAMPLING_INDEX]);
}

short GetTemperature(void)
{
  return ComputeDigital12BitsToTemperature(AdcAverage[TEMPERATURE_SAMPLING_INDEX]);
}
unsigned short GetOutputVoltage(void)
{
  return ComputeDigital12bitsToOutputVoltageSample(AdcAverage[OP_VOL_SAMPLING_INDEX]);
}
//DAC
unsigned short OutputVoltageToDigital12Bits(unsigned short targetVoltage)
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
unsigned short OpVolCalibration(unsigned short originalValue)
{
  return (originalValue*OutVolCalParaA/OP_VOL_PARA_KA+OutVolCalParaB/OP_VOL_PARA_KB)<0?0:(originalValue*OutVolCalParaA/OP_VOL_PARA_KA+OutVolCalParaB/OP_VOL_PARA_KB);
}
//CAN
char ReceivedCanCommendFlag=0;

unsigned int GetLocalCanId(void)
{
	return LOCAL_DEFAULT_ID;
}
unsigned int GetDeviceId(void)
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
  sFilterConfig.FilterIdHigh = (((unsigned int)HOST_CAN_ID<<21)&0xFFFF0000)>>16;
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
unsigned char ReceivedDebugCommandFlag=0;
void SetReceivedDebugCommandFlag(unsigned char receivedDebugCommandFlag){ ReceivedDebugCommandFlag=receivedDebugCommandFlag;}
unsigned char GetReceivedDebugCommandFlag(){  return ReceivedDebugCommandFlag;}
void CAN_ReciveDataHandler(CAN_HandleTypeDef *hcan)
{
  if((CONFIG_COMMAND==hcan->pRxMsg->Data[0])
    &&(false==GetBanFlashOperateFlag()))
  {
    SetReceivedDebugCommandFlag(true);
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
  SetDacOutputValue(OutputVoltageToDigital12Bits((ValidRxMessage.Data[2]<<8)|ValidRxMessage.Data[1]));
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
#define ADDR_FLASH_PAGE_127   ((unsigned int)0x0801FC00) /* Base @ of Page 127, 1 Kbytes */

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_127   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_127 + FLASH_PAGE_SIZE   /* End @ of user Flash area */
/* Private variables ---------------------------------------------------------*/
unsigned int *pPowerOnCounter=(unsigned int *)ADDR_FLASH_PAGE_127;
unsigned char BanFlashOperateFlag=false;
unsigned char GetBanFlashOperateFlag(void)
{
  return BanFlashOperateFlag;
}
unsigned int PAGEError = 0;
__IO unsigned int MemoryProgramStatus = 0;

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
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (unsigned int)pPowerOnCounter, ++powerOnCounter) != HAL_OK)
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
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (unsigned int)pPowerOnCounter, 1) != HAL_OK)
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













