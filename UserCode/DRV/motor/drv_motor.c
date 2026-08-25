//============================================================================================================================================================================
//                                              Motor_Driver[电机驱动]
//      包含了一些电机驱动相关的封装好的函数
//============================================================================================================================================================================


////////////////////////////////////////////////////////////////////
//                                                               //
// 修改了CANID后接收数据不用动，需要在发送函数处根据ID重新排序 //
//                                                               //
////////////////////////////////////////////////////////////////////
/*===| 底盘电机通信相关定义 |===*/
#define CHASSIS_MOTOR_CAN                   hfdcan1  //底盘CAN
#define CHASSIS_MOTOR_SEND_CAN_ID           0x200    //底盘电机CAN发送ID
#define CHASSIS_MOTOR1_FEEDBACK_CAN_ID      0x201    //底盘电机1CAN反馈ID
#define CHASSIS_MOTOR2_FEEDBACK_CAN_ID      0x202    //底盘电机2CAN反馈ID
#define CHASSIS_MOTOR3_FEEDBACK_CAN_ID      0x203    //底盘电机3CAN反馈ID
#define CHASSIS_MOTOR4_FEEDBACK_CAN_ID      0x204    //底盘电机4CAN反馈ID

/*===| 云台电机通信相关定义 |===*/
#define GIMBAL_YAW_CAN                      hfdcan2  //Yaw_使用的CAN
#define GIMBAL_YAW_SEND_CAN_ID              0x06     //Yaw_CAN发送ID
#define GIMBAL_YAW_FEEDBACK_CAN_ID          0x106    //Yaw_CAN反馈ID


#define GIMBAL_PITCH_CAN                    hfdcan3  //Pitch_使用的CAN
#define GIMBAL_PITCH_SEND_CAN_ID            0x1FE    //Pitch_CAN发送ID
#define GIMBAL_PITCH_FEEDBACK_CAN_ID        0x206    //Pitch_CAN反馈ID

/*===| 发射电机通信相关定义 |===*/
#define SHOOT_FRIC_CAN                      hfdcan3  //Fric_使用的CAN
#define SHOOT_FRIC_SEND_CAN_ID              0x200    //Fric_CAN发送ID
#define SHOOT_FRIC_RIGHT_FEEDBACK_CAN_ID    0x201    //右Fric_CAN反馈ID
#define SHOOT_FRIC_LEFT_FEEDBACK_CAN_ID     0x202    //左Fric_CAN反馈ID

#define SHOOT_TRIGGER_CAN                   hfdcan2  //Trigger_使用的CAN
#define SHOOT_TRIGGER_SEND_CAN_ID           0x200    //Trigger_CAN发送ID
#define SHOOT_TRIGGER_FEEDBACK_CAN_ID       0x204    //Trigger_CAN反馈ID

/*===| 超电通信相关定义 |==*/
#define SUPERCAP_CAN                        hfdcan1  //hcan1
#define SUPERCAP_CONTROL_CAN_ID             0x030    //超电控制CANID
#define SUPERCAP_FEEDBACK_CAN_ID            0x031    //超电反馈CANID

/*===| 陀螺仪通信相关定义 |==*/
#define IMU_CAN                             hfdcan3  //hcan3
#define IMU_CONTROL_CAN_ID                  0x001    //IMU控制CANID
#define IMU_FEEDBACK_CAN_ID                 0x002    //IMU反馈CANID

/*===| 遥控CAN转发通信相关定义 |==*/
#define REMOTE_CAN                          hfdcan3
#define REMOTE_CAN_JOYSTIC_CAN_ID           0x21
#define REMOTE_CAN_KEYBOARDMOUSE_CAN_ID     0x22

#include "drv_motor.h"

#include "drv_motor_dji.h"
#include "drv_motor_dm.h"
#include "fdcan.h"
#include "hal_can.h"
#include "hal_dwt.h"


Motor_HandleTypeDef hmotor_chassis1    = {};
Motor_HandleTypeDef hmotor_chassis2    = {};
Motor_HandleTypeDef hmotor_chassis3    = {};
Motor_HandleTypeDef hmotor_chassis4    = {};
Motor_HandleTypeDef hmotor_yaw         = {};
Motor_HandleTypeDef hmotor_pitch       = {};
Motor_HandleTypeDef hmotor_fric_right  = {};
Motor_HandleTypeDef hmotor_fric_left = {};
Motor_HandleTypeDef hmotor_trigger     = {};

Motor_HandleTypeDef *hmotor[MOTOR_COUNT] = {
        &hmotor_chassis1,
        &hmotor_chassis2,
        &hmotor_chassis3,
        &hmotor_chassis4,
        &hmotor_yaw,
        &hmotor_pitch,
        &hmotor_fric_right,
        &hmotor_fric_left,
        &hmotor_trigger
};


void Motor_Get_TotalAngle_Speed(Motor_HandleTypeDef *hmotor, float K)
{
        float Dt                           = DWT_GetDeltaT(&hmotor->Total_Angle_DWT_Count);
        hmotor->Total_Angle_Speed          = (hmotor->Total_Angle - hmotor->Total_Angle_Last) / Dt * 60.0f / 360.0f;
        hmotor->Total_Angle_Speed          = K * hmotor->Total_Angle_Speed + (1 - K) * hmotor->Total_Angle_Speed_RPM_Last;
        hmotor->Total_Angle_Speed_RPM_Last = hmotor->Total_Angle_Speed;

        hmotor->Total_Angle_Last = hmotor->Total_Angle;
}

//共九个电机初始化
static void Motor_Init()
{
        static const uint16_t Motor_CAN_Feedback_ID[9] = {
                CHASSIS_MOTOR1_FEEDBACK_CAN_ID,
                CHASSIS_MOTOR2_FEEDBACK_CAN_ID,
                CHASSIS_MOTOR3_FEEDBACK_CAN_ID,
                CHASSIS_MOTOR4_FEEDBACK_CAN_ID,
                GIMBAL_YAW_FEEDBACK_CAN_ID,
                GIMBAL_PITCH_FEEDBACK_CAN_ID,
                SHOOT_FRIC_RIGHT_FEEDBACK_CAN_ID,
                SHOOT_FRIC_LEFT_FEEDBACK_CAN_ID,
                SHOOT_TRIGGER_FEEDBACK_CAN_ID
        };
        //底盘电机 * 4
        for (int i = 0; i < 4; i++)
        {
                Motor_DJI_Ctor(hmotor[i], &CHASSIS_MOTOR_CAN, CHASSIS_MOTOR_SEND_CAN_ID, Motor_CAN_Feedback_ID[i]);
                Motor_Set_Status(hmotor[i], MOTOR_SPEED);
                PID_Init(
                         &hmotor[i]->PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }

        //yaw
        {
                Motor_DJI_Ctor(&hmotor_pitch, &GIMBAL_YAW_CAN, GIMBAL_YAW_SEND_CAN_ID, GIMBAL_YAW_FEEDBACK_CAN_ID);
                Motor_Set_Status(&hmotor_yaw, MOTOR_ANGLE);

                PID_Init(
                                 &hmotor_pitch.PID_Angle_Struct,
                                 32767,
                                 16384,
                                 0,
                                 1,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 Integral_Limit
                                );


                PID_Init(
                         &hmotor_pitch.PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         10,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }

        //pitch
        {
                Motor_DJI_Ctor(&hmotor_pitch, &GIMBAL_PITCH_CAN, GIMBAL_PITCH_SEND_CAN_ID, GIMBAL_PITCH_FEEDBACK_CAN_ID);
                Motor_Set_Status(&hmotor_pitch, MOTOR_ANGLE);

                PID_Init(
                         &hmotor_pitch.PID_Angle_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );

                PID_Init(
                         &hmotor_pitch.PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }

        //摩擦轮 * 2
        for (int i = 6; i < 8; i++)
        {
                Motor_DJI_Ctor(hmotor[i], &SHOOT_FRIC_CAN, SHOOT_FRIC_SEND_CAN_ID, Motor_CAN_Feedback_ID[i]);
                Motor_Set_Status(hmotor[i], MOTOR_SPEED);

                PID_Init(
                         &hmotor[i]->PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }


        //拨弹盘
        {
                Motor_DJI_Ctor(&hmotor_trigger, &SHOOT_TRIGGER_CAN, SHOOT_TRIGGER_SEND_CAN_ID,SHOOT_TRIGGER_FEEDBACK_CAN_ID);
                Motor_Set_Status(&hmotor_trigger, MOTOR_ANGLE);

                PID_Init(
                         &hmotor_trigger.PID_Angle_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );

                PID_Init(
                         &hmotor_trigger.PID_Speed_Struct,
                         32767,
                         16384,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         Integral_Limit
                        );
        }
}

void Motor_Control_Task(void *pvParameters)
{
        Motor_Init();

        for (;;)
        {
                //计算所有电机的pid
                for (int i = 0; i < MOTOR_COUNT; i++)
                {
                        //位置环
                        if (hmotor[i]->Status_Enum == MOTOR_ANGLE)
                        {
                                hmotor[i]->Target_Speed = PID_Calculate(&hmotor[i]->PID_Speed_Struct, hmotor[i]->Angle, hmotor[i]->Target_Angle);
                        }
                        //速度环
                        if (hmotor[i]->Status_Enum == MOTOR_ANGLE && hmotor[i]->Status_Enum == MOTOR_SPEED)
                        {
                                hmotor[i]->Target_Torque = PID_Calculate(&hmotor[i]->PID_Speed_Struct, hmotor[i]->Speed, hmotor[i]->Target_Speed);
                        }
                }

                //底盘电机发电流 can1
                Motor_DJI_SendCurrent(&CHASSIS_MOTOR_CAN, CHASSIS_MOTOR_SEND_CAN_ID,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis1),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis2),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis3),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_chassis4)
                                     );

                //yaw电机发电流 can2
                Motor_DM_Send_Torque(&hmotor_yaw, Motor_Get_Target_Torque(&hmotor_yaw));

                //拨弹盘 can2
                Motor_DJI_SendCurrent(&SHOOT_TRIGGER_CAN, SHOOT_TRIGGER_SEND_CAN_ID,
                                      0,
                                      0,
                                      0,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_trigger)
                                     );

                //pitch电机发电流 can3
                Motor_DJI_SendCurrent(&GIMBAL_PITCH_CAN, GIMBAL_PITCH_SEND_CAN_ID,
                                                      0,
                                                      (int16_t) Motor_Get_Target_Torque(&hmotor_pitch),
                                                      0,
                                                      0
                                                     );

                //俩摩擦轮 can3
                Motor_DJI_SendCurrent(&SHOOT_FRIC_CAN, SHOOT_FRIC_SEND_CAN_ID,
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_fric_right),
                                      (int16_t) Motor_Get_Target_Torque(&hmotor_fric_left),
                                      0,
                                      0
                                     );

                //ps：g4的每个can控制器都只有3个邮箱，所以不可以一次塞入3个以上个包
                vTaskDelay(1);
        }
}
