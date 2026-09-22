//
// Created by ye on 2026/8/26.
// 通用错误(超时/心跳)检测机制: 与硬件平台无关, 位于依赖链最底层
// 机制在本层(对象定义 + 注册表 + 遍历接口), 策略(周期任务/告警/恢复)在 SRV 层
//

#ifndef G4MINI_V3_ERR_H
#define G4MINI_V3_ERR_H

#include "stdint.h"
#include "stddef.h"
#include "cmsis_gcc.h"

#define ERR_MAX 255  //最大错误对象数

typedef struct Err_HandleTypeDef Err_HandleTypeDef;

typedef void (*err_handler)(Err_HandleTypeDef *herror);

struct Err_HandleTypeDef{
        volatile uint16_t tick;      //当前时间
        uint16_t tick_timeout;       //超时时间

        volatile uint16_t count;     //当前错误次数
        uint16_t count_maximum;      //错误次数上限

        volatile uint8_t If_Err;     //是否错误

        err_handler handler;         //错误处理回调
};

//错误检查
__STATIC_INLINE uint8_t Is_Err(Err_HandleTypeDef *herr) { return herr->If_Err; }

/*===| 机制API |===*/

//错误对象注册/注销
void Err_Register(Err_HandleTypeDef *herr, err_handler handler);
void Err_UnRegister(Err_HandleTypeDef *herr);

//错误对象构造(字段初始化 + 注册, If_Err 默认为 1)
void Err_Ctor(Err_HandleTypeDef *herr, uint16_t err_tick_Timeout, uint16_t err_count_maximum, err_handler handler);

/*===| 注册表遍历(供上层周期任务推进计时与聚合故障) |===*/
uint8_t            Err_Get_Count(void);
Err_HandleTypeDef *Err_Get_Handle(uint8_t index);

#endif //G4MINI_V3_ERR_H
