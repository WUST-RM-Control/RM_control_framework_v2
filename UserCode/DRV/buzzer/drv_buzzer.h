//
// Created by ye on 2026/8/26.
//

#ifndef G4MINI_V3_BUZZER_H
#define G4MINI_V3_BUZZER_H

/*===| 预设音效枚举定义 |===*/
typedef enum
{
        Buzzer_SoundEffect_OFF = 0,
        Buzzer_SoundEffect_SystemStart,
        Buzzer_SoundEffect_SuperCap_ON,
        Buzzer_SoundEffect_SuperCap_OFF,
        Buzzer_SoundEffect_Aim_ON,
        Buzzer_SoundEffect_Aim_OFF,
        Buzzer_SoundEffect_Speed1,
        Buzzer_SoundEffect_Speed2,
        Buzzer_SoundEffect_Speed3,
        Buzzer_SoundEffect_Speed4,
        Buzzer_SoundEffect_Speed5,
        Buzzer_SoundEffect_Shoot1,
        Buzzer_SoundEffect_Shoot2,
        Buzzer_SoundEffect_Shoot3,
        Buzzer_SoundEffect_Shoot4,
        Buzzer_SoundEffect_Shoot5,
        Buzzer_SoundEffect_Shoot6,
        Buzzer_SoundEffect_Shoot7,
        Buzzer_SoundEffect_Shoot8,
} Buzzer_SoundEffect_EnumTypedef;

extern Buzzer_SoundEffect_EnumTypedef Buzzer_SoundEffect;

/*===| 蜂鸣器音调枚举定义 |===*/
typedef enum
{
        P = 0,
        L1, L2, L3, L4, L5, L6, L7,
        M1, M2, M3, M4, M5, M6, M7,
        H1, H2, H3, H4, H5, H6, H7,
} Buzzer_Tone_EnumTypedef;


typedef struct
{
        TIM_HandleTypeDef* htim;
        uint8_t channel;

        //蜂鸣器音效任务间隔，10ms。建议不要高于30。
        TickType_t buzzer_task_tick;
} Buzzer_HandleTypeDef;


void Buzzer_Init(Buzzer_HandleTypeDef *hbuzzer);

void Buzzer_Set_SoundEffect(Buzzer_HandleTypeDef *hbuzzer, Buzzer_SoundEffect_EnumTypedef SoundEffect);

void Buzzer_Set_Tone(Buzzer_HandleTypeDef *hbuzzer, Buzzer_Tone_EnumTypedef Tone);

void Buzzer_Start(Buzzer_HandleTypeDef *hbuzzer);

void Buzzer_Stop(Buzzer_HandleTypeDef *hbuzzer);

__STATIC_INLINE TickType_t Buzzer_Get_Task_Tick(Buzzer_HandleTypeDef *hbuzzer)
{
        return hbuzzer->buzzer_task_tick;
}

#endif //G4MINI_V3_BUZZER_H
