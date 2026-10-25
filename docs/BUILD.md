# 构建、测试与复现

## 外部依赖

- NXP SDK 26.06 FRDM-MCXA346的完整west工作区（包含mcuxsdk、.west等）。
- Arm GNU工具链、CMake、Ninja；MCUXpresso扩展用于SDK配置。
- Python脚本使用Conda `py310`，需numpy、matplotlib、pyserial。
- 主机C一致性测试还使用WSL GCC和sanitizer；缺失时不能称为全套测试通过。
- J-Link烧录可选，执行会复位MCU并丢失RAM校准。本次整理不执行烧录。

## 本地配置

复制 `mcux_include.example.json` 为 `mcux_include.json` 并填写本机路径。
后者被Git忽略，避免公开个人安装目录；本机整理时已配置。
`SdkRootDirPath` 环境项指向**包含mcuxsdk的目录**，
CMake缓存项 `SdkRootDirPath` 则指向其中的mcuxsdk目录。
SDK配置过程沿用NXP自带的MCUX Python工具环境；训练、CLI与测试使用py310。

```powershell
cmake --preset debug
cmake --build --preset debug
conda activate py310
python tools/run_host_tests.py
```

生成 `debug/test1.elf` / `debug/test1.bin`；不再创建build、build2等目录。
VS Code打开本仓库根目录，默认构建任务使用debug预设。
SDK未随仓库复制，不能把仓库称为无外部依赖的离线包。

## 已编译的板级文件归属

既有工程实际编译的是SDK的
`examples/_boards/frdmmcxa346/board.c`、`clock_config.c`、
`common/pin_mux/pin_mux.c`，而非旧工程根目录的board文件夹。
自定义UART/I2C/PWM时钟与引脚初始化在app中。
旧board副本仅存放 `docs/history/board/` 供溯源，不参与构建。

## 结果复现（不启动电机）

```powershell
python tools/report_reversal.py data/reference25_confirm_20261023.csv
python tools/verify_dynamic.py data/reference25_confirm_20261023.csv --model data/dynamic_lookahead_linear_20261023.npz --friction-model data/friction_candidate_20261023_friction.json
```

仓库仅带精选数据，历史分析脚本可能需要完整旧数据集；缺失会明确报错，
不要据此伪造或跳过结果。完整原始实验仍在原开发目录。

## 烧录

确认电机功率断开、构建成功，核对工具路径与探针序列号后才可手动执行
`python tools/flash_jlink.py`。它只从当前仓库的debug取ELF，
不会再读取原工程目录的同名文件。实际接电机测试需另行按演示步骤执行。
