#ifndef __POWER_MODULE_H
#define __POWER_MODULE_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_can.h"
#include "can.h"
#include "stm32f1xx_hal_gpio.h"

#define POWER_ON_COMMAND        (0x01)
#define POWER_OFF_COMMAND       (0x00)
#define QUERY_COMMAND           (0x02)
#define FEEDBACK_COMMAND        (0x03)
#define ERROR_COMMAND           (0x04)

#define POWER_DEVICE            (0x00)


#define VDD_APPLI                      (2.5f)   /* Value of analog voltage supply Vdda (unit: V) */
#define EXTERNAL_VDD_APPLI             ((uint32_t) 6000)   
#define RANGE_12BITS                   ((uint32_t) 4095)   /* Max value with a full range of 12 bits */
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
enum{false,true};
enum{POWER_ON,POWER_OFF};
enum{LED_OFF,LED_ON};
extern char ReceivedCanCommendFlag;
extern __IO uint16_t AdcAverage[4];
extern uint16_t TargetOutputVoltage;
extern uint16_t CurInputVoltage;
extern uint16_t CurOutputVoltage;
extern uint16_t CurOutputCurrent;
extern int16_t CurModuleTemperature;
extern char InputOverVoltageFlag;
extern char InputUnderVoltageFlag;
extern char OutputOverVoltageFlag;
extern char OutputOverCurrentFlag;
extern char ShortCircuitFlag;
extern char OverTemperatureFlag;
extern char PowerStatusFlag;
extern uint32_t DacOutputValue;
extern uint32_t PowerOnDelayCounter;
extern uint32_t ShortCircuitRecoveryDelayCounter;
extern char MaybeShortCircuitFlag;
extern uint32_t ShortCircuitCheckDelayCounter;
extern CanRxMsgTypeDef        ValidRxMessage;


char  GetPowerStatus(void);
void FaultLightControl(GPIO_PinState lightState);
void PowerControl(GPIO_PinState PowerState);
GPIO_PinState GetPowerOutStatus(void);

//DAC
uint16_t OutputVoltageToDigital12Bits(float targetVoltage);
//ADC
void ADC_ExInit(ADC_HandleTypeDef* hadc);
uint16_t GetOutputCurrent(void);
uint16_t GetInputVoltage(void);
int16_t GetTemperature(void);
uint16_t GetOutputVoltage(void);
//CAN
uint32_t GetLocalCanId(void);
uint32_t GetDeviceId(void);
void CAN_ReciveDataHandler(CAN_HandleTypeDef *hcan);
void CAN_ExInit(CAN_HandleTypeDef *hcan);

void GetPowerModuleStatus(void);
#ifdef __cplusplus
}
#endif
#endif
