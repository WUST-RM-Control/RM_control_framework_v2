# UART 错误回调可扩展化 + 遥控器实例解耦

## Context

### 问题 1：单一弱符号回调无法扩展
`UserCode/DRV/remote/drv_remote.c:69` 用 HAL 弱符号 `HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)` 实现串口错误恢复，
内部靠 `if (hremote_dt7.huart == huart)` 判断。

- 弱符号全工程只能有一份实现。三个串口设备（remote/vofa/debug）目前共用这一个函数，
  只有 remote 在里面有分支；将来新增串口设备只能继续往同一个函数里堆 `if (huart == ...)`，互相耦合。
- `Core/Inc/stm32g4xx_hal_conf.h:107` 已经是 `USE_HAL_UART_REGISTER_CALLBACKS 1U`，
  即 HAL 支持 **per-handle 注册**，弱符号方案是历史遗留，不是必须。

三个串口设备现状：

| 设备 | 串口 | 用法 | 现有错误处理 |
|---|---|---|---|
| remote | `huart1` | DMA 空闲接收 (`HAL_UARTEx_ReceiveToIdle_DMA`) + `RegisterRxEventCallback` | 弱符号 `HAL_UART_ErrorCallback` + `hremote_dt7` 判断 |
| vofa | `huart2` | DMA 发送 (`HAL_UART_Transmit_DMA`) | 无 |
| debug | `huart3` | 中断发送 (`HAL_UART_Transmit_IT`，printf 重定向) | 无 |

### 问题 2：回调里写死了遥控器实例
弱符号回调内引用 `hremote_dt7` 具体实例。ENT 里换成 `hremote_vt03`
（或将来同时装配两个遥控器）后，回调仍然只看 `hremote_dt7`，与装配脱节。

### 目标
- remote / vofa / debug **三个串口设备各自注册**错误回调（而非弱符号覆盖），互不干扰，新增设备可无限扩展。
- 回调体不引用任何具体实例，改为跟随 ENT 的装配（`Remote_Ctor`/`VOFA_Ctor`/`Debug_Ctor` → `Xxx_Init()` 决定注册到哪个串口）。

## Approach

复用 HAL 已有的 per-handle 回调机制，与工程中已经在用的 `HAL_UART_RegisterRxEventCallback`
（`drv_remote.c:117`）保持完全对称：

```
Remote_Init(hremote)                       <- ENT 决定用哪个实例/哪个串口
  ├─ HAL_UART_RegisterRxEventCallback(hremote->huart, Remote_RxEvent_CallBack)   已有
  ├─ HAL_UART_RegisterCallback(hremote->huart, HAL_UART_ERROR_CB_ID, Remote_UART_Error_Callback)   新增
  └─ HAL_UARTEx_ReceiveToIdle_DMA(hremote->huart, ...)

出错时 HAL 内部: huart->ErrorCallback(huart)   <- 该句柄自己的回调, 只看 huart 参数
  └─ Remote_UART_Error_Callback(huart)
       └─ Remote_Restart_Receive(huart)        <- 只操作 huart, 不引用实例
```

关键点：回调签名 `void (*)(UART_HandleTypeDef *)` 不带 context，因此回调**只依赖 huart 参数**，
不保存也不查找实例 —— 这正是"随 ENT 改动"的实现方式（注册点决定归属，回调体保持中立）。

将来新增串口设备只需在自己 `Xxx_Init()` 里同样注册，不动其他设备一行代码 —— 这就是"增加更多串口设备"的答案。

三个设备的注册点与回调职责（HAL 错误分支只复位 **Rx** 侧 `UART_EndRxTransfer`，Tx 侧不动，
见 `stm32g4xx_hal_uart.c:2300`；Tx DMA 错误经 `UART_DMAError` 回调进入同一入口）：

| 设备 | 注册点 | 回调体动作 | 理由 |
|---|---|---|---|
| remote | `Remote_Init(hremote)` | `Remote_Restart_Receive(huart)` | 阻塞错误已 `UART_EndRxTransfer`，必须重挂 DMA 接收，否则永久失联 |
| vofa | `VOFA_Init()` | 错误计数（`VOFA_Error_Count++`） | 纯发送设备，DMA 层已复位，下次 `VOFA_Send_Data` 自然重发；仅记录供调试 |
| debug | `Debug_Init()` | 错误计数（`Debug_Error_Count++`） | 纯发送设备，同上；作为统一模式兼兜底 |

三个回调均为模块内 `static`，签名统一为 `static void Xxx_UART_Error_Callback(UART_HandleTypeDef *huart)`，
回调体内**只用传入的 `huart` 参数**，不引用任何具体实例/模块静态句柄。

## Files to modify

| 文件 | 改动 |
|---|---|
| `UserCode/DRV/remote/drv_remote.c` | 删除弱符号 `HAL_UART_ErrorCallback`；新增 `static Remote_UART_Error_Callback`；`Remote_Init` 内注册 + 防重入；`Remote_Restart_Receive`/`Remote_RxBuff` 收为 `static` |
| `UserCode/DRV/remote/drv_remote.h` | 删除 `Remote_Restart_Receive` 声明（已改 `static`，回调为模块内部实现，头文件不再导出） |
| `UserCode/DRV/vofa/drv_vofa.c` | 新增 `static VOFA_UART_Error_Callback` + `VOFA_Error_Count`；`VOFA_Init()` 内注册 |
| `UserCode/DRV/vofa/drv_vofa.h` | 新增 `uint32_t VOFA_Get_Error_Count(void);` |
| `UserCode/HAL/hal_debug.c` | 新增 `static Debug_UART_Error_Callback` + `Debug_Error_Count`；`Debug_Init()` 内注册（含 `Debug_UART == NULL` 守卫） |
| `UserCode/HAL/hal_debug.h` | 新增 `uint32_t Debug_Get_Error_Count(void);` |

> 无新增文件，CMake GLOB 无需重新 configure。

## Reuse

- `HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, pCallback)` — `Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c:730`，
  回调落到 `huart->ErrorCallback`，HAL 错误路径调用点 `:2330/:2343/:2356`。要求 `gState == HAL_UART_STATE_READY`（Init 后成立）。
- `HAL_UART_RegisterRxEventCallback(huart, pCallback)` — `:963`，已在本工程使用，模式完全一致。
- `Remote_Restart_Receive(huart)` — `drv_remote.c:61`，已只依赖 huart，直接复用为回调体。
- `HAL_UARTEx_ReceiveToIdle_DMA` / `__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT)` — 现有重挂接收写法。

## Steps

- [ ] 1. `drv_remote.c`：删除弱符号实现 `HAL_UART_ErrorCallback`（`drv_remote.c:69-75`）。
- [ ] 2. `drv_remote.c`：新增回调
      ```c
      /* UART错误回调: 由 Remote_Init 注册到 hremote->huart
       * 只依赖 huart 参数, 不引用具体实例, 因此随 ENT 装配变化 */
      static void Remote_UART_Error_Callback(UART_HandleTypeDef *huart)
      {
              Remote_Restart_Receive(huart);
      }
      ```
- [ ] 3. `drv_remote.c`：`Remote_Restart_Receive`（`:61`）与 `Remote_RxBuff`（`:21`）改为 `static`（无外部引用，`grep` 已确认）。
- [ ] 4. `drv_remote.c`：`Remote_Init` 中在 `HAL_UART_RegisterRxEventCallback` 旁注册错误回调
      ```c
      HAL_UART_RegisterCallback(hremote->huart, HAL_UART_ERROR_CB_ID, Remote_UART_Error_Callback);
      ```
- [ ] 5. `drv_remote.c`：`Remote_Init` 开头加防重入守卫（队列为模块单例，重复调用会重复建队列 + 任务）
      ```c
      if (Remote_RxQueue != NULL) return;   //已启动过, 避免重复创建队列/任务
      ```
- [ ] 6. `drv_remote.h`：删除 `void Remote_Restart_Receive(UART_HandleTypeDef *huart);` 声明，更新相关注释。
- [ ] 7. `drv_vofa.c/.h`：新增错误计数 + 回调，`VOFA_Init()` 内注册
      ```c
      static volatile uint32_t VOFA_Error_Count = 0;

      //UART错误回调: 发送侧错误, HAL/DMA 已复位状态, 此处仅计数
      static void VOFA_UART_Error_Callback(UART_HandleTypeDef *huart) { VOFA_Error_Count++; }

      void VOFA_Init()
      {
              memcpy(VOFA_message + VOFA_TXDATA_SIZE * 4, VOFA_tail, 4);

              if (VOFA_UART != NULL)
                      HAL_UART_RegisterCallback(VOFA_UART, HAL_UART_ERROR_CB_ID, VOFA_UART_Error_Callback);
      }

      uint32_t VOFA_Get_Error_Count(void) { return VOFA_Error_Count; }
      ```
- [ ] 8. `hal_debug.c/.h`：对 debug(`huart3`) 做同样处理
      ```c
      static volatile uint32_t Debug_Error_Count = 0;

      //UART错误回调: 发送侧错误, 仅计数
      static void Debug_UART_Error_Callback(UART_HandleTypeDef *huart) { Debug_Error_Count++; }

      void Debug_Init(void)
      {
              if (Debug_UART != NULL)
                      HAL_UART_RegisterCallback(Debug_UART, HAL_UART_ERROR_CB_ID, Debug_UART_Error_Callback);
      }

      uint32_t Debug_Get_Error_Count(void) { return Debug_Error_Count; }
      ```
- [ ] 9. 三个回调命名/签名统一为 `static void Xxx_UART_Error_Callback(UART_HandleTypeDef *huart)`，核对无弱符号残留。
- [ ] 10. 编译：`cmake --preset Debug && cmake --build build/Debug`，确认无 warning/error。

## 已确认的取舍

- **Q1 → A（最小改动）**：`Remote_RxBuff` / `Remote_RxQueue` / `Remote_RxTask` 保持模块单例，本次不实例化。
- **Q2 → 一并修改 remote / vofa / debug**：三个串口设备均改为 per-handle 注册错误回调，
  remote 做重挂接收，vofa/debug 做错误计数（各自 `Xxx_Get_Error_Count()` 可查）。
- **Q3 → 加防重入守卫**：`Remote_Init` 首次调用后 `Remote_RxQueue != NULL`，再次调用直接返回。

### 已知限制（A 方案的边界，明确记录）

- **不支持两个遥控器实例并发**：DT7 + VT03 同时启用会共用同一个 `Remote_RxBuff[64]` 和 `Remote_RxQueue`，
  且防重入守卫会直接挡掉第二次 `Remote_Init`（设计上就是单实例运行）。
  将来若需要 DT7 + VT03 同时工作，按 B 方案演进：`RxBuff`/`RxQueue` 移入 `Remote_HandleTypeDef`，
  并加 `huart → hremote` 映射表供 ISR 反查实例（可仿 `hal_can.c` 的 `hcan_node_table` 风格）。
- 回调注册本身已支持多串口，限制仅在缓冲/队列的资源归属上。

## Verification

1. **编译**：`cmake --build build/Debug` 零 warning。
2. **静态检查**：
   - `grep -rn "HAL_UART_ErrorCallback" UserCode/` → 无弱符号实现残留。
   - `grep -rn "hremote_dt7" UserCode/DRV/remote/` → 仅剩实例定义/注释，回调体内不出现。
   - `grep -rn "HAL_UART_ERROR_CB_ID" UserCode/` → 命中 3 处（remote / vofa / debug）。
   - `grep -rn "VOFA_Get_Error_Count\|Debug_Get_Error_Count" UserCode/` → 接口已导出。
3. **运行时（有硬件时）**：用调试器查看 `huart1/2/3.ErrorCallback` 分别指向
   `Remote_UART_Error_Callback` / `VOFA_UART_Error_Callback` / `Debug_UART_Error_Callback`（均非默认弱符号）。
4. **防重入**：连续调用两次 `Remote_Init(&hremote_dt7)`，确认只创建一个 `RemoteRx` 任务（FreeRTOS 任务列表）且 `Remote_RxQueue` 不泄漏。
5. **错误恢复验证**：制造 UART 错误（拔插遥控器接收线 / 短时错配波特率的发送 / 发送超长帧触发 ORE），
   确认错误后接收能自动恢复（`Remote_Is_Err()` 在 100ms 内重新变 0，遥控数据继续更新）。
6. **回归（多串口互不干扰）**：三个句柄各自持有自己的 `ErrorCallback`，
   断点分别命中 `huart1/2/3` 时确认只进入对应设备的回调（不再出现集中判断）。
