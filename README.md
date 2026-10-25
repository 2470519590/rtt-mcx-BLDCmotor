# RT-Thread + MCXA346 智能电机控制实验平台

2026 RT-Thread 嵌入式大赛参赛仓库整理版。硬件为 FRDM-MCXA346、
AS5600、SimpleFOC Mini、7对极2804电机；12V母线，无电流采样。
控制在MCU本地运行，上位机以串口CLI操作，网页用于监控。

## 目录

```text
app/              固件：RTT移植、编码器、PWM、速度/位置控制、NN、串口
rtthread/         现有工程使用的RT-Thread内核与Cortex-M33端口
tools/            CLI、监控、训练、数据采集、离线分析和主机测试
data/             冻结模型、三批精选对照与原始串口证据
docs/             提交材料、构建说明、接线及历史技术记录
demo/             演示脚本与待补视频材料
.vscode/          基于CMake预设的构建任务
debug/            唯一本地构建目录，Git忽略，不随仓库提交
```

原开发目录 `E:\NXP\projs\test1` 完整保留；本仓库是独立整理副本，
不依赖原工程源码路径。SDK与工具链为外部依赖，不打包SDK安装目录。
当前保留 `test1` 构建目标名以兼容既有工具，不额外改固件。

## 开始

1. 阅读 [构建与复现](docs/BUILD.md)，配置本地SDK/工具链路径。
2. 阅读 [接线](docs/HARDWARE.md)。
3. CLI：在仓库根目录 `conda activate py310` 后运行
   `python tools/control_cli.py auto`。USB串口编号自动按适配器身份选择。
4. 离线测试：`python tools/run_host_tests.py`。此操作不连接硬件。
5. 提交进度见 [材料清单](docs/SUBMISSION.md)。

参赛项目说明见 [PROJECT_INTRO.md](docs/PROJECT_INTRO.md)，预留两张图片。
网页监测启动：`python tools/web_monitor.py`，在浏览器打开终端显示的本地地址。
网页只负责监测；CLI与网页请勿同时打开同一个串口。

## 已验证与未验证

已具备编码器读取、速度/位置控制、传统控制基线、轻量NN训练和固件推理通路。
最新冻结的25ms P项目标平滑对照覆盖60段、三轮、十种低速切换：
整段RMSE均值下降5.83%，首秒下降11.77%。
这是**传统控制改进，不是NN收益**；NN未获得稳定超过线性预测的优势。
位置环未在本轮重新验证，视频及完整参赛说明尚待整理。

详见 [实验索引](data/README.md)。
历史记录中的“当前固件”、路径和早期收益仅代表各自实验时点，
不能替代本次冻结结论。

没有电流采样，本项目不是闭环电流控制；电压限幅不能代替电源限流。
本次仅整理仓库，未烧录或运行电机。第三方授权与发布前待办见
[第三方说明](THIRD_PARTY_NOTICES.md)。
