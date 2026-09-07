# 武汉科技大学崇实战队主控 G4mini-v3 整车控制框架

> 施工中...

# QX_CK_G4mini

基于现框架重构分层，使用c语言模拟面向对象的整车控制框架

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug

cmake --preset Release
cmake --build --preset Release
```


## G4mini Pin Assignment

### QMI8658A:

|Hardware|Pin|Peripheral|
|---|---|---|
|CLK|SPI1_CLK/PA5|SPI1|
|MISO|SPI1_MISO/PA6|SPI1|
|MOSI|SPI1_MOSI/PA7|SPI1|
|CS|PB0|GPIO|
|INT1|PB1|GPIO|
|INT2|PB2|GPIO|

### LED:

|Hardware|Pin|Peripheral|
|---|---|---|
|R|TIM1_CH2N/PB14|TIM1|
|G|TIM1_CH3N/PB15|TIM1|
|B|TIM1_CH1/PA8|TIM1|

### Buzzer:

|Hardware|Pin|Peripheral|
|---|---|---|
|BUZZER|TIM16_CH1/PB4|TIM16|

### USB:

|Hardware|Pin|Peripheral|
|---|---|---|
|DM|USB_DM/PA11|USB|
|DP|USB_DP/PA12|USB|

### CAN1:

|Hardware|Pin|Peripheral|
|---|---|---|
|TX|FDCAN1_TX/PB9|FDCAN1|
|RX|FDCAN1_RX/PB8|FDCAN1|

### CAN2:

|Hardware|Pin|Peripheral|
|---|---|---|
|TX|FDCAN2_TX/PB6|FDCAN2|
|RX|FDCAN2_RX/PB5|FDCAN2|

### CAN3:

|Hardware|Pin|Peripheral|
|---|---|---|
|TX|FDCAN3_TX/PA15|FDCAN3|
|RX|FDCAN3_RX/PB3|FDCAN3|

### USART1:

|Hardware|Pin|Peripheral|
|---|---|---|
|TX|USART1_TX/PA9|USART1|
|RX|USART1_RX/PA10|USART1|

### USART2:

|Hardware|Pin|Peripheral|
|---|---|---|
|TX|USART2_TX/PA2|USART2|
|RX|USART2_RX/PA3|USART2|

### USART3:

|Hardware|Pin|Peripheral|
|---|---|---|
|TX|USART3_TX/PB10|USART3|
|RX|USART3_RX/PB11|USART3|