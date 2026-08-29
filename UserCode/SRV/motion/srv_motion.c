//
// Created by ye on 2026/8/28.
//

#include "srv_motion.h"

#include "drv_motor.h"

//运动解算
void Chassis_Ctrl(float Vx, float Vy, float W)
{
        float V_motor1 = 0;
        float V_motor2 = 0;
        float V_motor3 = 0;
        float V_motor4 = 0;

        //相量法还是太好用了（:D）
        V_motor1 += Vx;
        V_motor3 += Vy;

        V_motor2 += Vy;
        V_motor4 += Vy;

        V_motor1 += W;
        V_motor2 += W;
        V_motor3 += W;
        V_motor4 += W;

        //把速度直接给成力矩
        Motor_Set_Torque(&hmotor_chassis1, V_motor1);
        Motor_Set_Torque(&hmotor_chassis2, V_motor2);
        Motor_Set_Torque(&hmotor_chassis3, V_motor3);
        Motor_Set_Torque(&hmotor_chassis4, V_motor4);
}
