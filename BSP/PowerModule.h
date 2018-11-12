#ifndef __POWER_MODULE_H
#define __POWER_MODULE_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_can.h"
#include "can.h"
#include "stm32f1xx_hal_gpio.h"
   
#define VDD_APPLI                      (2.5f)   /* Value of analog voltage supply Vdda (unit: V) */
#define EXTERNAL_VDD_APPLI             ((uint32_t) 6000)   
#define RANGE_12BITS                   ((uint32_t) 4095)   /* Max value with a full range of 12 bits */
//can
#define LOCAL_CAN_ID                    (0x03)
#define HOST_CAN_ID                     (0x01)
   
//�����ѹ�����㷶Χ
#define IN_OVER_V_PROTECT_MAX       (670+10)  //680
#define IN_OVER_V_PROTECT_MIN       (670-10)  //660
//�����ѹ�ָ��㷶Χ
#define IN_OVER_V_RECOVER_MAX       (650+10)  //660
#define IN_OVER_V_RECOVER_MIN       (650-10)  //640
//����Ƿѹ�����㷶Χ
#define IN_UNDER_V_PROTECT_MAX       (460+10)  //470
#define IN_UNDER_V_PROTECT_MIN       (460-10)  //450
//����Ƿѹ�ָ��㷶Χ
#define IN_UNDER_V_RECOVER_MAX       (480+10)  //490
#define IN_UNDER_V_RECOVER_MIN       (480-10)  //470
//�����ѹ�����㷶Χ     
//����Դ��ֱ�������ѹֵ�ﵽ 48 V��0.5 V ʱ����ѹֵ����Ӧͣ����������������״̬��
//��ѹͣ�������Զ��ָ����������¿������ָܻ�������
#define OUT_OVER_V_PROTECT_MAX      (48+0.5f)
#define OUT_OVER_V_PROTECT_MIN      (48-0.5f)

//�������
//��Դ��������ﵽ 25 A��30 A��������ѹ��������Ͻ�����Զ��ָ�������
#define OUT_LIMIT_CURRENT_MAX      (30)
#define OUT_LIMIT_CURRENT_MIN      (25)
//��·����
//����Դ�����·ʱ����ԴӦͣ�����������Ͻ���Զ��ָ�������

//���±���
//����Դ�ڲ��¶ȴﵽ 95 ���5 ��ʱ����ԴӦ�ػ�����������Ϲ���ָʾ����������Դ
//�ڲ��¶Ƚ���70 ���5 ��ʱ����Դ�Զ��ָ�������
#define OVER_TEMPERATURE_VALUE    (900)
#define OVER_TEMPERATURE_CANCEL_VALUE   (750)
enum{false,true};

extern char ReceivedCanCommendFlag;
extern uint16_t AdcAverage[5];
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

uint16_t GetOutputCurrent(void);
uint16_t GetInputVoltage(void);
uint16_t GetTemperature(void);
uint16_t GetOutputVoltage(void);
char  GetPowerStatus(void);
void FaultLightControl(GPIO_PinState lightState);
void PowerControl(GPIO_PinState PowerState);
void CAN_ReciveDataHandler(CAN_HandleTypeDef hcan);
void CAN_ExtendedInit(CAN_HandleTypeDef *hcan);

void GetPowerModuleStatus(void);
#ifdef __cplusplus
}
#endif
#endif
