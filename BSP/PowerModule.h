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

   
#define VDD_APPLI                      (2.5f)   /* Value of analog voltage supply Vdda (unit: V) */
#define EXTERNAL_VDD_APPLI             ((uint32_t) 6000)   
#define RANGE_12BITS                   ((uint32_t) 4095)   /* Max value with a full range of 12 bits */
//can
#define LOCAL_CAN_ID                    (0x03)
#define HOST_CAN_ID                     (0x01)
   
//输入过压保护点范围
#define IN_OVER_V_PROTECT_MAX       (670)  //680
#define IN_OVER_V_PROTECT_MIN       (670)  //660
//输入过压恢复点范围
#define IN_OVER_V_RECOVER_MAX       (650)  //660
#define IN_OVER_V_RECOVER_MIN       (650)  //640
//输入欠压保护点范围
#define IN_UNDER_V_PROTECT_MAX       (460)  //470
#define IN_UNDER_V_PROTECT_MIN       (460)  //450
//输入欠压恢复点范围
#define IN_UNDER_V_RECOVER_MAX       (480)  //490
#define IN_UNDER_V_RECOVER_MIN       (480)  //470
//输出过压保护点范围     
//当电源的直流输出电压值达到 48 V±0.5 V 时（过压值），应停机保护并锁定保护状态，
//过压停机后不能自动恢复，必须重新开机才能恢复工作。
#define OUT_OVER_V_PROTECT_MAX      (48)
#define OUT_OVER_V_PROTECT_MIN      (48)

//输出限流
//电源输出电流达到 25 A～30 A，限流降压输出，故障解除后自动恢复工作。
#define OUT_LIMIT_CURRENT_MAX      (30)
#define OUT_LIMIT_CURRENT_MIN      (26)
//短路保护
//当电源输出短路时，电源应停机保护，故障解除自动恢复工作。

//过温保护
//当电源内部温度达到 95 ℃±5 ℃时，电源应关机保护，面板上过温指示灯亮；当电源
//内部温度降至70 ℃±5 ℃时，电源自动恢复工作。
#define OVER_TEMPERATURE_VALUE    (900)
#define OVER_TEMPERATURE_CANCEL_VALUE   (750)
enum{false,true};

extern char ReceivedCanCommendFlag;
extern uint16_t AdcAverage[4];
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


char  GetPowerStatus(void);
void FaultLightControl(GPIO_PinState lightState);
void PowerControl(GPIO_PinState PowerState);

//ADC
void ADC_ExInit(ADC_HandleTypeDef* hadc);
uint16_t GetOutputCurrent(void);
uint16_t GetInputVoltage(void);
uint16_t GetTemperature(void);
uint16_t GetOutputVoltage(void);
//CAN
void CAN_ReciveDataHandler(CAN_HandleTypeDef hcan);
void CAN_ExInit(CAN_HandleTypeDef *hcan);

void GetPowerModuleStatus(void);
#ifdef __cplusplus
}
#endif
#endif
