# 当前硬件与接线

| MCXA346 | 外设 | 用途 |
|---|---|---|
| P3_0 / PWM0_A0 / PWM_AT | 驱动逻辑输入A | 电机M1 |
| P3_8 / PWM0_A1 / PWM_BT | 驱动逻辑输入B | 电机M2 |
| P3_10 / PWM0_A2 / PWM_CT | 驱动逻辑输入C | 电机M3 |
| J5-5 / P3_27 / LPI2C3_SCL | AS5600 SCL | 400kHz I2C |
| J5-6 / P3_28 / LPI2C3_SDA | AS5600 SDA | 地址0x36 |
| J5-4 / P4_5 / LPUART3 TX | 外置USB-TTL RX | 115200 |
| J5-3 / P4_2 / LPUART3 RX | 外置USB-TTL TX | 115200 |
| GND | 驱动、编码器、串口模块GND | 共地 |

AS5600使用3.3V供电与上拉，串口模块使用3.3V逻辑，不接模块VCC到板上。
驱动母线12V，EN固定高；无硬件电流采样。PWM引脚接驱动逻辑输入，
不能接电机绕组。当前使用外置CH343，不再依赖板载USB VCOM。
J-Link用于MCXA346 SWD，不能把J-Link CDC串口当作电机通信串口。

待补：实物照片、驱动板丝印与供电图、整机固定方式。
