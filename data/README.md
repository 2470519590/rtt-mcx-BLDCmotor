# 精选模型与实测证据

保留原文件名以兼容分析脚本。未把全部历史数据和runs目录复制进参赛仓库。

| 前缀 | 用途 | 结论 |
|---|---|---|
| reference25_confirm_20261023 | 三轮60段P参考25ms独立确认 | 整段均值RMSE改善5.83%，不是NN收益 |
| transition_integral_ab_20261023 | 三轮60段积分reset/keep对照 | 保留积分平均退步，默认reset |
| dynamic_reversal_ab_20261023 | 三轮90段静态/动态NN/线性对照 | 没有稳定NN增量优势 |
| reference25_final_stop_20261023_uart | 收尾原始串口记录 | 停止、无故障，build3005 |

每批包含CSV、采集元数据、分项报告、曲线、电压核对，以及`_uart/`内
raw.bin、reads.jsonl、writes.jsonl。原始字节与时间日志用于独立重放，
不是应删除的调试垃圾。元数据中的绝对路径是**历史采集来源**，
本次不改写历史事实；离线报告用当前输入文件旁的日志。

模型：

- friction_candidate_20261023_friction.json：简单静态电压修正。
- friction_candidate_20261023_friction_nn.json：原静态NN对照模型。
- dynamic_lookahead_linear_20261023.npz/.json：冻结动态NN/线性模型。
- angle_residual_candidate_v2.npz：旧角度模型测试夹具，不代表本轮有效收益。

详细结论与限制见 [实测结果](../docs/RESULTS.md)。
