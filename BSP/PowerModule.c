#include "PowerModule.h"

uint16_t AdcAverage[4];
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
char PowerStatusFlag=0;
uint32_t DacOutputValue=0;

//OUTPUT
void FaultLightControl(GPIO_PinState lightState)
{
  HAL_GPIO_WritePin(FAULT_LIGHT_GPIO_Port, FAULT_LIGHT_Pin, lightState);
}
void WorkLightControl(GPIO_PinState lightState)
{
  HAL_GPIO_WritePin(WORK_LIGHT_GPIO_Port, WORK_LIGHT_Pin, lightState);
}
void PowerControl(GPIO_PinState PowerState)
{
  if(1==(hcan1.pRxMsg->Data[0]&0x01))
  {
    HAL_GPIO_WritePin(POWER_CONTROL_GPIO_Port, POWER_CONTROL_Pin, PowerState);
  }
  else
  {
    HAL_GPIO_WritePin(POWER_CONTROL_GPIO_Port, POWER_CONTROL_Pin, GPIO_PIN_RESET);
  }
  
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
  ((uint16_t)((( (ADC_DATA) * VDD_APPLI / RANGE_12BITS *4.0f)+0.4f)*120.0f*10.0f))

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
                        sizeof(AdcAverage)
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

uint16_t GetTemperature(void)
{
  return AdcAverage[2];
}
uint16_t GetOutputVoltage(void)
{
  return COMPUTATION_DIGITAL_12BITS_TO_OUTPUT_VOLTAGE_SAMPLE(AdcAverage[3]);
}
//DAC
uint16_t OutputVoltageToDigital12Bits(uint16_t targetVoltage)
{
  if(targetVoltage>463)
  {
    return 0;
  }
  else
  {
    return (int16_t)((targetVoltage*(-0.002298)+1.064)/VDD_APPLI*RANGE_12BITS);
  }
}
//CAN
char ReceivedCanCommendFlag=0;

void CAN_ExInit(CAN_HandleTypeDef* hcan)
{
  static CanTxMsgTypeDef        TxMessage;
  static CanRxMsgTypeDef        RxMessage;
  //CAN_FilterConfTypeDef  sFilterConfig;
  /*## Configure the CAN Filter ###########################################*/
//  sFilterConfig.FilterNumber = 0;
//  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  sFilterConfig.FilterIdHigh = 0x0000;
//  sFilterConfig.FilterIdLow = 0x0000;
//  sFilterConfig.FilterMaskIdHigh = 0x0000;
//  sFilterConfig.FilterMaskIdLow = 0x0000;
//  sFilterConfig.FilterFIFOAssignment = 0;
//  sFilterConfig.FilterActivation = ENABLE;
//  sFilterConfig.BankNumber = 14;

//  if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
//  {
//    /* Filter configuration Error */
//    Error_Handler();
//  }
  /*## Configure Transmission process #####################################*/
  hcan->pTxMsg = &TxMessage;
  hcan->pRxMsg = &RxMessage;
  
  hcan->pTxMsg->StdId=HOST_CAN_ID;
  hcan->pTxMsg->ExtId = 0x01;
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
void CAN_ReciveDataHandler(CAN_HandleTypeDef hcan)
{
  if(LOCAL_CAN_ID==hcan.pRxMsg->StdId)
  {
    ReceivedCanCommendFlag=true;
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
  DacOutputValue=OutputVoltageToDigital12Bits((hcan1.pRxMsg->Data[2]<<8)|hcan1.pRxMsg->Data[1]);
  if(CurInputVoltage>=IN_OVER_V_PROTECT_MIN)
  {
    InputOverVoltageFlag=true;
  }
  else
  {
    InputOverVoltageFlag=false;
  }
  
  if(CurInputVoltage<=IN_UNDER_V_RECOVER_MIN)
  {
    InputUnderVoltageFlag=true;
  }
  else
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
  
  if(CurModuleTemperature>=OVER_TEMPERATURE_VALUE)
  {
    OverTemperatureFlag=1;
  }
  else if(CurModuleTemperature<=OVER_TEMPERATURE_CANCEL_VALUE)
  {
    OverTemperatureFlag=0;
  }
  
  //¶ÌÂ·ÅÐ¶Ï
}