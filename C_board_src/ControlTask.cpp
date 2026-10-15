#include "ControlTask.h"
#include "Gm6020.h"
#include "Pid.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"
#include <cmath>
namespace {
constexpr float    kPi         = 3.14159265358979323846f;
constexpr float    kRpmToRadS  = (2.0f * kPi) / 60.0f;
}

#define USE_VEL_MODE 1
Gm6020 motor(1);
Pid pid_pos(500.0f, 5.0f, 5.0f, 16384.0f,-16384.0f);
Pid pid_vel(500.0f, 0.0f, 0.0f, 16384.0f,-16384.0f);
volatile uint32_t g_ms=0;
volatile float vel_ref=0.0f;
volatile float pos_ref=0.0f;
volatile float out=0.0f;
volatile float motor_vel=0.0f;
volatile float motor_angle=0.0f;
CAN_HandleTypeDef *hcan=&hcan1; 

 //配置CAN过滤器、启动CAN与定时器
void ControlTaskInit(void) {
    CAN_FilterTypeDef filter={0};
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterBank = 0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter)!= HAL_OK) {
        Error_Handler();
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(hcan->Instance == CAN1) {
        CAN_RxHeaderTypeDef Rx;
        uint8_t data[8];        
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx, data);
        CanFeedbackCallback(Rx.StdId, data);}
}

void MainTask(void) {
    g_ms++;
    HAL_IWDG_Refresh(&hiwdg);

    //速度跟随0.3Hz、幅值50rpm
    vel_ref=50.0f*sin(2.0f*kPi*0.3f*g_ms*0.001f)*kRpmToRadS;

    //位置跟随0.2Hz、幅值2π
    static float angle0=0.0f;
    static uint8_t pos_inited=0;
    if(!pos_inited&&g_ms>100){
        angle0=motor.angle();
        pos_inited=1;
    }
    pos_ref=angle0+2.0f*kPi*std::sin(2.0f*kPi*0.2f*g_ms*0.001f);

    //电流给定
    motor_vel=motor.vel();
    motor_angle=motor.angle();
    if(USE_VEL_MODE){
        out=pid_vel.calc(vel_ref,motor_vel);
    }
    else{
        out=pid_pos.calc(pos_ref,motor_angle);
    }
    motor.setCurrent((int16_t)out);

    // 发送控制帧
    uint8_t data[8]={0};
    motor.encode(data);
    CAN_TxHeaderTypeDef Tx={0};
    Tx.StdId = motor.txId();
    Tx.DLC = 8;
    Tx.IDE = CAN_ID_STD;
    Tx.RTR = CAN_RTR_DATA;
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(hcan, &Tx, data, &mailbox);
}

void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id == motor.rxId()) {
        motor.decode(data);
    }
}
