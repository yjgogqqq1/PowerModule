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

#define HOST_CAN_ID                     (0x01)
   
//�����ѹ�����㷶Χ
#define IN_OVER_V_PROTECT_MAX       (6700)  //680
#define IN_OVER_V_PROTECT_MIN       (6700)  //660
//�����ѹ�ָ��㷶Χ
#define IN_OVER_V_RECOVER_MAX       (6500)  //660
#define IN_OVER_V_RECOVER_MIN       (6500)  //640
//����Ƿѹ�����㷶Χ
#define IN_UNDER_V_PROTECT_MAX       (4600)  //470
#define IN_UNDER_V_PROTECT_MIN       (4600)  //450
//����Ƿѹ�ָ��㷶Χ
#define IN_UNDER_V_RECOVER_MAX       (4800)  //490
#define IN_UNDER_V_RECOVER_MIN       (4800)  //470
//�����ѹ�����㷶Χ     
//����Դ��ֱ�������ѹֵ�ﵽ 48 V��0.5 V ʱ����ѹֵ����Ӧͣ����������������״̬��
//��ѹͣ�������Զ��ָ����������¿������ָܻ�������
#define OUT_OVER_V_PROTECT_MAX      (480)
#define OUT_OVER_V_PROTECT_MIN      (480)

//�������
//��Դ��������ﵽ 25 A��30 A��������ѹ��������Ͻ�����Զ��ָ�������
#define OUT_LIMIT_CURRENT_MAX      (300)
#define OUT_LIMIT_CURRENT_MIN      (260)


//��·����
//����Դ�����·ʱ����ԴӦͣ�����������Ͻ���Զ��ָ�������

//���±���
//����Դ�ڲ��¶ȴﵽ 95 ���5 ��ʱ����ԴӦ�ػ�����������Ϲ���ָʾ����������Դ
//�ڲ��¶Ƚ���70 ���5 ��ʱ����Դ�Զ��ָ�������
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
void CAN_ReciveDataHandler(CAN_HandleTypeDef *hcan);
void CAN_ExInit(CAN_HandleTypeDef *hcan);

void GetPowerModuleStatus(void);
#ifdef __cplusplus
}
#endif
#endif
