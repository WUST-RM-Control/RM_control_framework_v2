#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "drv_buzzer.h"

Buzzer_SoundEffect_EnumTypedef Buzzer_SoundEffect;

Buzzer_HandleTypeDef hbuzzer1 = {};

void Buzzer_Ctor(Buzzer_HandleTypeDef *hbuzzer, TIM_HandleTypeDef *htim, uint8_t Buzzer_TIM_Channel, TickType_t buzzer_task_control_time)
{
        hbuzzer->htim = htim;
        hbuzzer->channel = Buzzer_TIM_Channel;

        hbuzzer->buzzer_task_tick = buzzer_task_control_time;
}

void Buzzer_Init(Buzzer_HandleTypeDef *hbuzzer)
{
        /*===| 启动蜂鸣器PWM |===*/
        HAL_TIM_Base_Start(hbuzzer->htim);
        HAL_TIM_PWM_Start(hbuzzer->htim, hbuzzer->channel);
}

void Buzzer_Start(Buzzer_HandleTypeDef *hbuzzer)
{
        HAL_TIM_PWM_Start(hbuzzer->htim, hbuzzer->channel);
}

void Buzzer_Stop(Buzzer_HandleTypeDef *hbuzzer)
{
        HAL_TIM_PWM_Stop(hbuzzer->htim, hbuzzer->channel);
}

void Buzzer_Task(void const *argument)
{
        Buzzer_Init(&hbuzzer1);
        for (;;)
        {
                /*===| 根据需要播放的声音来分别播放音符 |===*/
                switch ((uint8_t) Buzzer_SoundEffect)
                {
                case Buzzer_SoundEffect_OFF :
                {
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        Buzzer_Stop(&hbuzzer1);
                        break;
                }
                case Buzzer_SoundEffect_SystemStart :
                {
                        Buzzer_Start(&hbuzzer1);
                        Buzzer_Set_Tone(&hbuzzer1, M1);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, M2);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, M3);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, M2);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, M5);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, M7);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, M6);
                        vTaskDelay(500);
                        Buzzer_Set_Tone(&hbuzzer1, M5);
                        vTaskDelay(250);
                        Buzzer_Set_Tone(&hbuzzer1, M6);
                        vTaskDelay(750);
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_SuperCap_ON :
                {
                        Buzzer_Start(&hbuzzer1);
                        Buzzer_Set_Tone(&hbuzzer1, M1);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, M3);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, M5);
                        vTaskDelay(200);
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_SuperCap_OFF :
                {
                        Buzzer_Start(&hbuzzer1);
                        Buzzer_Set_Tone(&hbuzzer1, M5);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, M3);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, M1);
                        vTaskDelay(200);
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Aim_ON :
                {
                        Buzzer_Start(&hbuzzer1);
                        Buzzer_Set_Tone(&hbuzzer1, M7);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(150);
                        Buzzer_Set_Tone(&hbuzzer1, M7);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(150);
                        Buzzer_Set_Tone(&hbuzzer1, M7);
                        vTaskDelay(50);
                        Buzzer_Set_Tone(&hbuzzer1, P);
                        vTaskDelay(150);
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Aim_OFF :
                {
                        Buzzer_Start(&hbuzzer1);
                        Buzzer_Set_Tone(&hbuzzer1, M7);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, M5);
                        vTaskDelay(200);
                        Buzzer_Set_Tone(&hbuzzer1, M3);
                        vTaskDelay(200);
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Shoot1 :
                {
                        Buzzer_Start(&hbuzzer1);
                        for (uint8_t i = 0; i < 1; i++)
                        {
                                Buzzer_Set_Tone(&hbuzzer1, M5);
                                vTaskDelay(100);
                                Buzzer_Set_Tone(&hbuzzer1, P);
                                vTaskDelay(100);
                        }
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Shoot2 :
                {
                        Buzzer_Start(&hbuzzer1);
                        for (uint8_t i = 0; i < 2; i++)
                        {
                                Buzzer_Set_Tone(&hbuzzer1, M5);
                                vTaskDelay(100);
                                Buzzer_Set_Tone(&hbuzzer1, P);
                                vTaskDelay(100);
                        }
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Shoot3 :
                {
                        Buzzer_Start(&hbuzzer1);
                        for (uint8_t i = 0; i < 3; i++)
                        {
                                Buzzer_Set_Tone(&hbuzzer1, M5);
                                vTaskDelay(100);
                                Buzzer_Set_Tone(&hbuzzer1, P);
                                vTaskDelay(100);
                        }
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Shoot4 :
                {
                        Buzzer_Start(&hbuzzer1);
                        for (uint8_t i = 0; i < 4; i++)
                        {
                                Buzzer_Set_Tone(&hbuzzer1, M5);
                                vTaskDelay(100);
                                Buzzer_Set_Tone(&hbuzzer1, P);
                                vTaskDelay(100);
                        }
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Shoot5 :
                {
                        Buzzer_Start(&hbuzzer1);
                        for (uint8_t i = 0; i < 5; i++)
                        {
                                Buzzer_Set_Tone(&hbuzzer1, M5);
                                vTaskDelay(100);
                                Buzzer_Set_Tone(&hbuzzer1, P);
                                vTaskDelay(100);
                        }
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Shoot6 :
                {
                        Buzzer_Start(&hbuzzer1);
                        for (uint8_t i = 0; i < 6; i++)
                        {
                                Buzzer_Set_Tone(&hbuzzer1, M5);
                                vTaskDelay(100);
                                Buzzer_Set_Tone(&hbuzzer1, P);
                                vTaskDelay(100);
                        }
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Shoot7 :
                {
                        Buzzer_Start(&hbuzzer1);
                        for (uint8_t i = 0; i < 7; i++)
                        {
                                Buzzer_Set_Tone(&hbuzzer1, M5);
                                vTaskDelay(100);
                                Buzzer_Set_Tone(&hbuzzer1, P);
                                vTaskDelay(100);
                        }
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                case Buzzer_SoundEffect_Shoot8 :
                {
                        Buzzer_Start(&hbuzzer1);
                        for (uint8_t i = 0; i < 8; i++)
                        {
                                Buzzer_Set_Tone(&hbuzzer1, M5);
                                vTaskDelay(100);
                                Buzzer_Set_Tone(&hbuzzer1, P);
                                vTaskDelay(100);
                        }
                        Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                        break;
                }
                }

                vTaskDelay(Buzzer_Get_Task_Tick(&hbuzzer1));
        }
}

/**
 * @brief 设置蜂鸣器音效
 */
void Buzzer_Set_SoundEffect(Buzzer_HandleTypeDef *hbuzzer, Buzzer_SoundEffect_EnumTypedef SoundEffect)
{
        Buzzer_SoundEffect = SoundEffect;
}

/**
 * @brief 设置蜂鸣器音调
 */
void Buzzer_Set_Tone(Buzzer_HandleTypeDef *hbuzzer, Buzzer_Tone_EnumTypedef Tone)
{


        uint16_t PSC;
        uint32_t ARL;
        uint32_t System_Fequency = HAL_RCC_GetSysClockFreq();

        switch ((uint8_t) Tone)
        {
        case P :
                PSC = 50;
                ARL = 100;
                break;
        case L1 :
                PSC = 50;
                ARL = System_Fequency / PSC / 262;
                break;
        case L2 :
                PSC = 50;
                ARL = System_Fequency / PSC / 294;
                break;
        case L3 :
                PSC = 50;
                ARL = System_Fequency / PSC / 330;
                break;
        case L4 :
                PSC = 50;
                ARL = System_Fequency / PSC / 349;
                break;
        case L5 :
                PSC = 50;
                ARL = System_Fequency / PSC / 392;
                break;
        case L6 :
                PSC = 50;
                ARL = System_Fequency / PSC / 440;
                break;
        case L7 :
                PSC = 50;
                ARL = System_Fequency / PSC / 494;
                break;
        case M1 :
                PSC = 50;
                ARL = System_Fequency / PSC / 523;
                break;
        case M2 :
                PSC = 50;
                ARL = System_Fequency / PSC / 587;
                break;
        case M3 :
                PSC = 50;
                ARL = System_Fequency / PSC / 659;
                break;
        case M4 :
                PSC = 50;
                ARL = System_Fequency / PSC / 698;
                break;
        case M5 :
                PSC = 50;
                ARL = System_Fequency / PSC / 784;
                break;
        case M6 :
                PSC = 50;
                ARL = System_Fequency / PSC / 880;
                break;
        case M7 :
                PSC = 50;
                ARL = System_Fequency / PSC / 988;
                break;
        case H1 :
                PSC = 50;
                ARL = System_Fequency / PSC / 1046;
                break;
        case H2 :
                PSC = 50;
                ARL = System_Fequency / PSC / 1175;
                break;
        case H3 :
                PSC = 50;
                ARL = System_Fequency / PSC / 1318;
                break;
        case H4 :
                PSC = 50;
                ARL = System_Fequency / PSC / 1397;
                break;
        case H5 :
                PSC = 50;
                ARL = System_Fequency / PSC / 1568;
                break;
        case H6 :
                PSC = 50;
                ARL = System_Fequency / PSC / 1760;
                break;
        case H7 :
                PSC = 50;
                ARL = System_Fequency / PSC / 1976;
                break;
        default : //P
                PSC = 50;
                ARL = 100;
                break;
        }
        __HAL_TIM_SET_PRESCALER(hbuzzer->htim, PSC - 1);
        __HAL_TIM_SET_AUTORELOAD(hbuzzer->htim, ARL - 1);
        __HAL_TIM_SetCompare(hbuzzer->htim, hbuzzer->channel, ARL / 2);
}
