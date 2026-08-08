# panthera_pose_tuner（已停用）

此包是旧的 MoveIt 点位调试工具，仅为历史兼容保留。生产一键启动不会加载它，禁止
与 `panthera_motion` Motion Server 或生产状态机同时运行。

日常示教请使用常驻 Web HMI 的独占调试模式。点位保存到
`src/panthera_motion/config/motion_catalog.yaml`，通过原子保存、重新编译和热重载生效。
操作步骤见仓库文档
[`docs/SPECTROMETER_CELL_POINT_TUNING.md`](../../docs/SPECTROMETER_CELL_POINT_TUNING.md)。
