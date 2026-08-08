# 旧 panthera_pose_tuner 说明（已停用）

`panthera_pose_tuner` 不再用于生产调点，也不会由一键启动或 HMI 启动。它依赖旧的
MoveIt 在线规划和硬编码点位；与固定轨迹 Motion Server 同时运行会形成多个命令源。

当前点位调试统一在常驻 HMI 中完成：

1. 打开 `http://127.0.0.1:8080`。
2. 进入“点位调试”，点击“进入调试（不移动）”。
3. 需要示教时选择点位并点击“前往所选点位”。
4. 使用 XYZ/RPY 单轴点动或直接输入坐标。
5. 保存后由后端原子写入 `motion_catalog.yaml`、编译并热重载；失败自动回滚。

生产点位唯一数据源是：

```text
src/panthera_motion/config/motion_catalog.yaml
```

详细操作见 [SPECTROMETER_CELL_POINT_TUNING.md](SPECTROMETER_CELL_POINT_TUNING.md)。
旧包源码仅保留用于历史追溯；不要按旧流程启动真实机械臂。
