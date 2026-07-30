# 项目级 Agent 入口

- 当任务涉及 `master/` 下的速度环、巡线直线段、巡线弧线/转向段、角度环、VOFA 遥测或 PID 调参时，开始工作前必须完整读取 `master/PID_TUNING_WORKDOC.md` 和 `master/PID_TUNING_STATE.md`。
- 调参会话只以源码、车上实际 `PARAMS`、本轮采集配置和完整原始日志为证据；不得依赖聊天记忆猜测遥测索引、固件版本或参数值。
- 每轮有效试跑后，把结果追加到 `master/tuning_log.jsonl`；在收到配对的 ARM/DONE 数据前不得创建虚假试跑记录。
- `master/PID_TUNING_WORKDOC.md` 定义的允许修改区、禁止修改区和人工操作边界优先适用于上述调参任务。

