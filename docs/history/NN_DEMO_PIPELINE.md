# 传统控制 + 轻量神经网络 Demo

当前实现保留传统速度/位置控制作为基线，NN 只作为可切换的残差项：

```text
base_uq = 速度前馈 + 速度 PI
nn_delta = MLP(target_speed, speed, error, target_position, position, base_uq)
uq = clamp(base_uq + alpha * nn_delta)
```

固件默认 `nn off`、`alpha=0`、学习遥测关闭；因此烧录后仍是之前验证过的传统控制。`nn shadow` 会计算并输出 NN 残差但不改变 PWM，`nn on` 才会按照 `nnalpha` 叠加。

## 采集

先确认 AS5600 已上电、校准已经成功，再打开串口。自动采集脚本会依次做速度阶跃和小位置阶跃，数据只使用 `sensor_ok=1`、`calibrated=1`、无故障且未饱和的帧：

```powershell
E:\miniconda\envs\py310\python.exe tools\collect_nn_dataset.py COM32 data\nn_demo.csv --seconds-per-step 4
```

只采速度：

```powershell
E:\miniconda\envs\py310\python.exe tools\collect_nn_dataset.py COM32 data\nn_speed.csv --seconds-per-step 5 --no-position
```

脚本不会启动 HTTP 服务，也不会自动烧录；它只打开指定外部 USB-UART，发送协议命令并记录 `L,` 学习帧。需要保持驱动、电机和传感器处于可运行状态。

## 训练与导出

训练脚本使用 NumPy 的 6-8-1 ReLU MLP，教师目标是一个很小的速度误差残差，避免网络替代传统控制：

```powershell
E:\miniconda\envs\py310\python.exe tools\train_nn.py data\nn_speed.csv --output data\nn_speed_model.npz
E:\miniconda\envs\py310\python.exe tools\export_nn_weights.py data\nn_speed_model.npz --output app\nn_model.h
ninja -C debug
```

`app/nn_model.h` 是编译输入，不要手工编辑；重新导出会覆盖它。导出前后必须保留一份模型归档和同名 `.json` 训练指标。

## 实机验证顺序

先烧录并确认传统基线：

```text
stop
nn off
learn off
calibrate
run 60
state
```

再验证 shadow：

```text
stop
learn on
nn shadow
nnalpha 0
run 60
state
```

确认 `L,` 帧正常、`nn_mode=1`，且导出前的中性模型 `nn_delta_uq_mv=0`。最后才逐步尝试：

```text
stop
nn on
nnalpha 0.10
run 60
state
```

出现跟踪变差时直接 `stop`、`nn off`，恢复传统基线。当前是 Demo pipeline，不把离线拟合结果当作正式性能或安全结论。
