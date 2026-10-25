# test1：FRDM-MCXA346 编码器读取起始工程

本目录由 MCUXpresso SDK 26.06 中的 `demo_apps/mc_pmsm/pmsm_enc` 复制而来，保留了 NXP 已生成并验证过的底层文件，暂时不需要使用 MCUXpresso Config Tools。

当前这个 bring-up 版本包含：

- RT-Thread 与 USB Type-C / MCU-Link VCOM（LPUART2，115200 baud）
- mikroBUS J5 的 LPI2C3，连接 AS5600（7-bit 地址 `0x36`）
- 只读编码器监控：USB VCOM 查询与本地 Web 仪表盘
- AS5600 RAW ANGLE、角度（毫弧度/厘度）及角速度读取

当前固件包含低电压电压模式 FOC 速度 P 控制。电机为 7 对极，母线 12 V，SimpleFOC Mini EN 固定高。上电零占空比，必须在网页校准后明确启动才产生驱动输出。

## 编译

本工程使用 MCUXpresso SDK 的标准 CMake 入口和 Ninja。SDK 的 Arm GCC
工具链路径需要通过 `ARMGCC_DIR` 提供：

```powershell
$env:ARMGCC_DIR = 'E:/STM32CubeCLT_1.21.0/GNU-tools-for-STM32'
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=debug
cmake --build build
```

生成物位于 `build/test1.elf` 和 `build/test1.bin`。如果更换工具链或 SDK
配置，建议删除 `build` 后重新执行上述命令。

## AS5600 接线

| FRDM-MCXA346 | AS5600 |
|---|---|
| mikroBUS J5-5 / P3_27 | SCL |
| mikroBUS J5-6 / P3_28 | SDA |
| GND | GND |
| 3.3 V 电源 | VCC |

I2C 线上需要合适的 3.3 V 上拉。确认模块供电和引脚标注后再连接；不要把 5 V 上拉接到 MCU IO。

## USB VCOM Web 监控

在 Windows 终端使用已有 `py310` Conda 环境运行本地 Web 服务（环境中需有 `pyserial`）：

```powershell
conda activate py310
python .\tools\web_monitor.py
```

先下载 `debug/test1.elf` 的新固件。串口启动提示应为 `encoder telemetry ready; period_ms=10; format=T`。关闭串口助手，避免与 Web 服务同时占用 COM 口。

浏览器打开 <http://127.0.0.1:18765>，刷新串口列表后选择 MCU-Link VCOM 对应的 COM 口并连接。默认波特率为 115200；可用 `--baud` 修改波特率，用 `--port` 修改网页服务端口。若端口不可用，以终端打印的实际网址为准。

仪表盘显示 AS5600 机械角度、RAW ANGLE、角速度、弧度及数据新鲜度，并用指针和转子动画呈现角度变化。固件每 10 ms 主动发送紧凑遥测，目标 100 Hz；Web 后端只接收，通过 SSE 长连接推送最新值，不积压历史帧。页面使用 requestAnimationFrame 刷新，物理显示频率受显示器限制。数据格式为 `T,序号,RTT毫秒tick,原始角度,毫弧度每秒,传感器有效位`：

```text
T,42,500,2048,0,1
```

RT-Thread 约每 10 ms 采样一次 AS5600；遥测发送最近一次采样值。Web 工具不会初始化电机或启用 PWM。按 `Ctrl+C` 停止本地 Web 服务。

当前 100 Hz 是监控阶段目标，不是未来速度环带宽承诺。闭环必须在 MCU 本地执行，控制采样率通常按闭环带宽的 10～20 倍起步，再根据延迟、滤波和稳定裕度验证；网页不参与控制环。页面接收频率为实测值，数据 age 仅表示到达 Python 后的时间，不等于端到端测量延迟。

## 安全边界

### 初次转动测试

1. 接线：P3_0/PWM0_A0 -> 驱动 A，P3_8/PWM0_A1 -> 驱动 B，P3_10/PWM0_A2 -> 驱动 C；逻辑地共地。PWM 只能接逻辑输入，不能接电机绕组。
2. 使用 12 V 限流电源，电机空载固定好，准备随时物理断电。没有电流采样，软件电压限幅不能防止过流。
3. 下载新 `debug/test1.elf`，运行固件，连接网页串口。初态必须显示停止、未校准、Uq=0。
4. CLI直接连接串口，不依赖网页：先在网页断开串口，再执行 `python tools/control_cli.py COM27`。网页监控和CLI不能同时打开同一个COM口。旧 `motor_cli.py` 是历史工具，不要同时运行。
5. 首先 `voltage 1.0`、`openloop 10` 验证转动，再 `stop`。开环无需校准，RPM是旋转磁场的机械等效速度，不保证转轴跟得上。负RPM反转；电压设置目前限0.1～1.5 V，只有停止时接受，不自动升压。若1.5 V仍不转，先排查PWM输出和驱动相序，不盲目加电压。
6. 确认开环能转后执行 `calibrate`；校准成功再 `kp 0.08`、`run 30` 测试P速度环。`state`读取模式、校准状态和故障码。低速/堵转自动判定已移除。`stop`停止输出；CLI退出请求停止，失去CLI心跳1秒也停止。网页隐藏不再控制电机。

软件停止不等于切断母线，EN固定高下驱动仍通电，零占空比可能是短接绕组制动，取决于驱动板真值表。没有电流采样，不能用软件电压限幅代替限流电源。

电角度方向是校准确定的正反方向，不保证等于观察者视角的顺/逆时针。控制任务目标500 Hz，实际速度估计使用采样tick差值，100 Hz遥测不参与闭环。当前实现未完成硬件电流保护、看门狗、UART DMA和运行中故障引脚处理；CPU停在调试断点时PWM硬件可能保持最后输出，严禁带功率调试暂停。驱动故障输入无接线，因此PWM内部未使用外部fault映射。

## 当前固件（nnphase1）新增的命令与工具

以下只影响 RAM，复位后回到编译进固件的模型；**角度补偿（G6）仍为试验模式，
本轮实测未发现可归因于角度模型的收益，不要当作推荐默认**（见
`MOTOR_OPTIMIZATION.md` 最新一节的对照实验）。

| 命令 | 作用 |
|---|---|
| `M<i>` / `u<milli>` / `p<milli>` | 选择归一化点、写相位（mrad）、提交该点的转速（mRPM） |
| `g<field>` | 一个输入权重：`20000 + index*2500 + |w|*1000`，index 必须连续 |
| `i<milli>` / `n0` | 神经元输出权重、提交该神经元 |
| `q<milli>` | 输出偏置（mV） |
| `m0..3` | bit0 内插开关，bit1 表示 g 使用紧凑字段 |
| `h<milli>` | 可选隐藏层偏置（1e-4 单位） |
| `x0` / `z0` | 只读：上传状态 `X,…,g_stage`、构建号 `3001` |
| `s<0..11>` | 只读：编译进固件的输入 std（毫单位），用于核对模型 |

自动化工具（均使用 `E:\miniconda\envs\py310\python.exe`，不依赖网页）：

```powershell
# 主机测试（沙箱内需要用包装脚本，见下）
E:\miniconda\envs\py310\python.exe tools\run_host_tests.py

# 编译并烧录（沙箱无法回显 ninja 子进程输出时用 python 版）
E:\miniconda\envs\py310\python.exe debug\_build.py
E:\miniconda\envs\py310\python.exe tools\flash_jlink.py

# 上机前恢复会话（只在确实未校准时才加 --calibrate-if-needed）
E:\miniconda\envs\py310\python.exe tools\prepare_motor_session.py --calibrate-if-needed

# 采集 NN-off 基线并拟合全局/每速度/延迟三种相位表
E:\miniconda\envs\py310\python.exe tools\optimize_motor.py --output data\refit.csv --configs pi_kp150_ki100 --repeats 3 --seconds 10 --targets 15 -15 30 -30 20 -20
E:\miniconda\envs\py310\python.exe tools\fit_angle_phase.py data\refit.csv --model data\angle_residual_candidate_v2.npz --output data\fit.json --holdout-speeds 20 -20

# 上传 RAM 模型并在 G5 影子模式下与 float32 参考核对（不施加输出）
E:\miniconda\envs\py310\python.exe tools\upload_angle_model.py --model data\angle_residual_candidate_v2.npz --phase 0.0 --shadow 15

# 四配置 A/B（含零相位对照）与“零权重网络”决定性对照
E:\miniconda\envs\py310\python.exe tools\angle_phase_ab.py --output data\ab.csv --model ... --phase-report data\fit.json
E:\miniconda\envs\py310\python.exe tools\confirm_angle_model.py --output data\confirm.csv --model data\angle_residual_candidate_v2.npz
```

`debug/_build.py` 会直接调用 ninja 记录下来的编译命令并比较对象文件哈希，
用于“构建成功但产物没变”的空构建陷阱；`debug/_check_build.py` 只做语法检查。
`debug/_host_test_tmp`、`debug/_tmptest` 是早期沙箱探测留下的空目录，
当前权限下删不掉（只能清空后 `rmdir`），可以忽略。
