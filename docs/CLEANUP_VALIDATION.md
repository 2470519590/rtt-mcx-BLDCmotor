# 仓库整理验收（2026-10-25）

目标：`E:\NXP\repos\rtt-mcx-motor`。
来源：`E:\NXP\projs\test1`，原目录保留且未编辑。

## 目录处理

- 独立初始化本地Git仓库；未提交commit、未配置remote、未上传。
- 保留app、RT-Thread、CLI/分析/训练工具和主机测试。
- 历史MD与未参与编译的board副本移到新仓库的docs/history。
- 选入三批实板对照和收尾记录，保留原始字节和read/write时间日志。
- 不复制旧debug、runs、legacy_foc、根目录零散CSV或Python缓存。
- debug为新路径实际验证时生成，受Git忽略；本机mcux_include.json同样忽略。
- 修正五个工具对旧工程的硬编码路径，未修改控制算法或模型。

## 本机验证

- app/和rtthread/逐文件SHA核对：相对来源零差异。
- `cmake --preset debug`：成功。
- `cmake --build --preset debug`：51项全量构建成功。
- 125项主机测试全部通过，0失败、0错误、0跳过。
- 在新目录重放reference25_confirm：26289帧连续、原指标一致；
  动态预测/静态施加电压核对通过。
- 二进制长度均73572字节；与旧工程bin仅8字节不同，全部位于RTT编译
  时间/日期字符串。旧对象日期Oct12，新全量构建Oct25。
  不把不同哈希说成字节完全一致。
- 新bin SHA256：
  `3DC239AAB53D8850A4624B370620D9734D5E6F5972C67F61C53CB78503CB3A2A`。
- SDK提示当前CMake3.28.1/Ninja1.11.1低于推荐版本，实际配置/构建仍成功。
  APP_DIR未使用警告来自沿用的SDK预设，不代表构建失败。

本次没有烧录、串口连接、校准或实际电机测试；移植后构建与离线测试
不等于新路径固件已经上板验证。历史数据仍代表原实验镜像。

## 剩余提交工作

正式参赛正文、实物图、软件框架图、演示视频、授权选择及上游版本溯源
均在SUBMISSION.md中列为待办，不能称为已提交或材料已全部完成。
