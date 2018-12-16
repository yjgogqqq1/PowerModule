#ifndef __POWER_MODULE_H
#define __POWER_MODULE_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_can.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_i2c.h"
#include "can.h"



#define POWER_ON_COMMAND        (0x01)
#define POWER_OFF_COMMAND       (0x00)
#define QUERY_COMMAND           (0x02)
#define RESET_FAULT_COMMAND     (0x04)

#define FEEDBACK_COMMAND        (0x03)
#define ERROR_COMMAND           (0x04)

//
#define CONFIG_COMMAND          (0x03)
#define POWER_DEVICE            (0x00)


#define VDD_APPLI                      (2.5f)   /* Value of analog voltage supply Vdda (unit: V) */
#define EXTERNAL_VDD_APPLI             ((unsigned int) 6000)   
#define RANGE_12BITS                   ((unsigned int) 4095)   /* Max value with a full range of 12 bits */
//can

#define HOST_CAN_ID                     (0x01)
#define BROADCAST_ID                    (0x00)
//输入过压保护点范围
#define IN_OVER_V_PROTECT_MAX       (6700)  //680
#define IN_OVER_V_PROTECT_MIN       (6700)  //660
//输入过压恢复点范围
#define IN_OVER_V_RECOVER_MAX       (6500)  //660
#define IN_OVER_V_RECOVER_MIN       (6500)  //640
//输入欠压保护点范围
#define IN_UNDER_V_PROTECT_MAX       (4600)  //470
#define IN_UNDER_V_PROTECT_MIN       (4600)  //450
//输入欠压恢复点范围
#define IN_UNDER_V_RECOVER_MAX       (4800)  //490
#define IN_UNDER_V_RECOVER_MIN       (4800)  //470
//输出过压保护点范围     
//当电源的直流输出电压值达到 48 V±0.5 V 时（过压值），应停机保护并锁定保护状态，
//过压停机后不能自动恢复，必须重新开机才能恢复工作。
#define OUT_OVER_V_PROTECT_MAX      (480)
#define OUT_OVER_V_PROTECT_MIN      (480)

//输出限流
//电源输出电流达到 25 A～30 A，限流降压输出，故障解除后自动恢复工作。
#define OUT_LIMIT_CURRENT_MAX      (300)
#define OUT_LIMIT_CURRENT_MIN      (260)


//短路保护
//当电源输出短路时，电源应停机保护，故障解除自动恢复工作。

//过温保护
//当电源内部温度达到 95 ℃±5 ℃时，电源应关机保护，面板上过温指示灯亮；当电源
//内部温度降至70 ℃±5 ℃时，电源自动恢复工作。
#define OVER_TEMPERATURE_VALUE    (950)
#define OVER_TEMPERATURE_CANCEL_VALUE   (750)
#define MAX_OUTPUT_V						(463)
#define MIN_OUTPUT_V						(245)
#define POWER_ON_MAX_DELAY					(1000)
#define SHORT_CIRCUIT_MAX_DELAY			(3000)
#define MAX_SHORT_CIRCUIT_CHECK_DELAY     (100)

//eeprom
#define EVAL_I2Cx_TIMEOUT_MAX                   3000
#define I2C_OWN_ADDRESS                            0x0A              // stm32本机I2C地址



enum{false,true};
typedef enum
{
  POWER_ON,
  POWER_OFF
} PowerStatus;
typedef enum
{
  LED_OFF,
  LED_ON
}LightStatus;

typedef struct
{
  unsigned int ErrorCode;
  unsigned int RunTotalTime;
} ErrorInfor;


extern char ReceivedCanCommendFlag;
extern __IO unsigned short AdcAverage[10];
extern unsigned short TargetOutputVoltage;
extern unsigned short CurInputVoltage;
extern unsigned short CurOutputVoltage[5];
extern unsigned short CurOutputCurrent[5];
extern short CurModuleTemperature;
extern char InputOverVoltageFlag;
extern char InputUnderVoltageFlag;
extern char OutputOverVolFlag[5];
extern char OutputOverCurFlag[5];
extern char ShortCircuitFlag;
extern char OverTemperatureFlag;
extern char FaultSendStopFlag;
extern char PowerStatusFlag;
extern unsigned int PowerOnDelayCounter;
extern unsigned int ShortCircuitRecoveryDelayCounter;
extern char MaybeShortCircuitFlag;
extern unsigned int ShortCircuitCheckDelayCounter;
extern CanRxMsgTypeDef        ValidRxMessage;

ErrorInfor *GetErrorInforListBaseAddr(void);
char  GetPowerStatus(void);
void FaultLightControl(LightStatus lightState);
GPIO_PinState GetFaultLightStatus(void);
void PowerControl(unsigned char index,PowerStatus powerStatus);
GPIO_PinState GetPowerOutStatus(void);

//DAC
unsigned short OutputVoltageToDigital12Bits(unsigned short targetVoltage);
//ADC
void ADC_ExInit(ADC_HandleTypeDef* hadc);
unsigned short GetOpVolSampling(unsigned char channel);
unsigned short GetOpCurSampling(unsigned char channel);
unsigned short GetInputVoltage(void);
short GetTemperature(void);
//CAN
unsigned int GetLocalCanId(void);
unsigned int GetDeviceId(void);
void SetReceivedDebugCommandFlag(unsigned char receivedDebugCommandFlag);
unsigned char GetReceivedDebugCommandFlag(void);
void CAN_ReciveDataHandler(CAN_HandleTypeDef *hcan);
void CAN_ExInit(CAN_HandleTypeDef *hcan);
void GetPowerModuleStatus(void);

//flash

unsigned char GetBanFlashOperateFlag(void);
void PowerOnCounterInc(void);
//eeprom
void SetReadErrorInforEnableFlag(char readErrorInforEnableFlag);
char GetReadErrorInforEnableFlag(void);
HAL_StatusTypeDef EepromWrite(I2C_HandleTypeDef *hi2c,unsigned short MemAddress, uint8_t *pData, unsigned short Size);
void ReadCriticalDataFromEeprom(I2C_HandleTypeDef *hi2c);
void DebugConmmandProcess(I2C_HandleTypeDef *hi2c,CAN_HandleTypeDef *pHcan);
void NewErrorInforSave(unsigned char errorCode,unsigned int runTotalTime);
unsigned int GetProgramRunMinutes(void);
void ProgramRunTiming(void);
#ifdef __cplusplus
}
#endif
#endif
