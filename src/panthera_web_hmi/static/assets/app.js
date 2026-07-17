const flowStates = [
  'INIT',
  'IDLE',
  'WAIT_DISCHARGE',
  'SELECT_TASK',
  'PICK_FROM_OUTLET',
  'MEASURE_SPECTROMETER_BEFORE_PLACE',
  'PLACE_TO_SPECTROMETER',
  'START_DETECTION',
  'WAIT_DETECTION_DONE',
  'MEASURE_SPECTROMETER_BEFORE_PICK',
  'PICK_FROM_SPECTROMETER',
  'CLEAN_CUP',
  'RETURN_CUP',
  'COMPLETE_CYCLE',
  'PAUSED',
  'ESTOP',
  'ERROR',
  'RESET',
];

const flowLabels = {
  INIT: '初始化',
  IDLE: '空闲',
  WAIT_DISCHARGE: '等待出料',
  SELECT_TASK: '选择任务',
  PICK_FROM_OUTLET: '出料口取杯',
  MEASURE_SPECTROMETER_BEFORE_PLACE: '放杯前测距',
  PLACE_TO_SPECTROMETER: '放到光谱仪',
  START_DETECTION: '启动检测',
  WAIT_DETECTION_DONE: '等待检测',
  MEASURE_SPECTROMETER_BEFORE_PICK: '取杯前测距',
  PICK_FROM_SPECTROMETER: '从光谱仪取杯',
  CLEAN_CUP: '清理杯子',
  RETURN_CUP: '放回出料口',
  COMPLETE_CYCLE: '本轮完成',
  PAUSED: '暂停',
  ESTOP: '急停',
  ERROR: '错误停机',
  RESET: '复位',
};

const actionStates = new Set([
  'PICK_FROM_OUTLET',
  'MEASURE_SPECTROMETER_BEFORE_PLACE',
  'PLACE_TO_SPECTROMETER',
  'START_DETECTION',
  'MEASURE_SPECTROMETER_BEFORE_PICK',
  'PICK_FROM_SPECTROMETER',
  'CLEAN_CUP',
  'RETURN_CUP',
  'RESET',
]);

const workflowCommands = new Set([
  'workflow_visible_motion_check',
  'workflow_fixed_large_motion_demo',
  'workflow_cup_pick_place',
  'workflow_outlet_1_pick_check_actual',
  'workflow_outlet_2_pick_check_actual',
  'workflow_spectrometer_place_check_actual',
  'workflow_clean_dump_check_actual',
  'workflow_arm_home',
  'workflow_gripper_open',
  'workflow_gripper_close',
]);

const safeStateRequestTargets = new Set(['IDLE', 'WAIT_DISCHARGE', 'PAUSED', 'RESET', 'ESTOP']);
const CAMERA_REFRESH_INTERVAL_MS = 33;

let lastState = null;
let eventSource = null;
let latestSnapshot = null;
let cameraMode = 'rgb';
let lastCameraRefreshMs = 0;
let estopClearAcknowledged = false;
let poseTunerDebugMode = false;
let poseTunerTargets = [];
let poseTunerBusy = false;
let speedScalePercent = 100;
let speedScaleDirty = false;
let speedScalePendingPercent = null;
let speedScalePendingUntilMs = 0;
let pointConfig = null;
let pointConfigDirty = false;
let pointConfigSelectedPose = '';
let activePage = localStorage.getItem('panthera_hmi_page') || 'dashboard';

const pointMotionLabels = {
  outlet_approach_y: '出料口入口 Y',
  outlet_near_y: '出料口靠近 Y',
  outlet_high_z: '出料口高位 Z',
  outlet_grip_z: '出料口夹取 Z',
  outlet_transfer_z: '取杯后抬高 Z',
  spectrometer_approach_x_offset: '光谱仪接近 X 偏移',
  spectrometer_high_z: '光谱仪高位 Z',
  clean_high_z: '清理高位 Z',
  clean_approach_y: '清理入口 Y',
  clean_pre_z: '清理低位 Z',
  clean_ready_z: '清理准备 Z',
};

const pointAxisLabels = {
  laser_min_mm: '激光最小 mm',
  laser_max_mm: '激光最大 mm',
  axis_zero_laser_mm: '基准激光 mm',
  axis_scale_m_per_mm: '轴换算 m/mm',
  axis: '移动轴 x/y/z',
  place_offset_xyz: '放杯补偿 xyz',
  pick_offset_xyz: '取杯补偿 xyz',
};

const pointCleaningLabels = {
  pour_wrist_joint_index: '倒料关节下标',
  pour_direction: '倒料旋转方向',
  pour_angle_rad: '倒料角 rad',
  pour_velocity_scale: '倒料速度',
  pour_acceleration_scale: '倒料加速度',
  pour_hold_sec: '倒料停留 s',
  shake_count: '摆动次数',
  shake_angle_rad: '摆动角 rad',
  shake_hold_sec: '摆动停留 s',
  brush_enabled: '启用毛刷清洁',
  brush_pose: '毛刷点位',
  brush_velocity_scale: '毛刷靠近速度',
  brush_acceleration_scale: '毛刷靠近加速度',
  brush_approach_offset_xyz: '毛刷进入偏移 xyz',
  brush_upright_retreat_offset_xyz: '毛刷回正前退出 xyz',
  brush_stroke_count: '毛刷往复次数',
  brush_stroke_offset_xyz: '毛刷往复偏移 xyz',
  brush_hold_sec: '毛刷停留 s',
  brush_motor_stop_delay_sec: '毛刷退出后电机延时 s',
  motor_serial_enabled: '清洁电机串口启用',
  motor_serial_device: '清洁电机串口设备',
  motor_serial_baudrate: '清洁电机波特率',
  motor_start_byte: "清洁电机启动字符码",
  motor_stop_byte: "清洁电机停止字符码",
};

function $(id) {
  return document.getElementById(id);
}

function setText(id, value) {
  const el = $(id);
  if (el) {
    el.textContent = value;
  }
}

function setActivePage(page) {
  const validPages = new Set(['dashboard', 'points', 'logs']);
  activePage = validPages.has(page) ? page : 'dashboard';
  localStorage.setItem('panthera_hmi_page', activePage);

  document.querySelectorAll('[data-page-target]').forEach((button) => {
    button.classList.toggle('active', button.dataset.pageTarget === activePage);
  });
  document.querySelectorAll('[data-page]').forEach((panel) => {
    panel.classList.toggle('page-hidden', panel.dataset.page !== activePage);
  });

  const layout = document.querySelector('.layout');
  if (layout) {
    layout.classList.remove('page-dashboard', 'page-points', 'page-logs');
    layout.classList.add(`page-${activePage}`);
  }

  if (activePage === 'points' && !pointConfig) {
    loadPointConfig();
  }
}

function boolText(value) {
  if (value === true) {
    return '是';
  }
  if (value === false) {
    return '否';
  }
  return '--';
}

function formatAge(age) {
  if (age === null || age === undefined || Number.isNaN(Number(age))) {
    return '--';
  }
  if (age < 1) {
    return `${Math.round(age * 1000)} ms`;
  }
  return `${Number(age).toFixed(1)} s`;
}

function formatMm(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return '--';
  }
  return `${Number(value).toFixed(1)} mm`;
}

function formatPoseMm(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return '--';
  }
  return `${Number(value).toFixed(2)} mm`;
}

function formatQuat(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return '--';
  }
  return Number(value).toFixed(6);
}

function formatDeg(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return '--';
  }
  return `${Number(value).toFixed(1)} deg`;
}

function formatQueue(value) {
  if (!value) {
    return '--';
  }
  if (Array.isArray(value)) {
    return value.length ? value.join(', ') : '空';
  }
  if (typeof value === 'object') {
    return Object.entries(value)
      .map(([key, val]) => `${key}:${val}`)
      .join(' / ') || '--';
  }
  return String(value);
}

function setConnection(ok) {
  const badge = $('connectionBadge');
  if (!badge) {
    return;
  }
  badge.textContent = ok ? '已连接' : '离线';
  badge.className = ok ? 'badge badge-ok' : 'badge badge-error';
}

function stateClass(state) {
  if (state === 'ERROR' || state === 'ESTOP') {
    return 'badge badge-error';
  }
  if (state === 'PAUSED' || state === 'WAIT_DISCHARGE' || state === 'WAIT_DETECTION_DONE') {
    return 'badge badge-warn';
  }
  if (state && state !== 'UNKNOWN') {
    return 'badge badge-ok';
  }
  return 'badge';
}

function deriveMode(context, state) {
  if (state === 'ESTOP') {
    return 'ESTOP';
  }
  if (state === 'ERROR') {
    return 'ERROR';
  }
  if (state === 'PAUSED' || context.pause_requested) {
    return 'PAUSED';
  }
  if (context.manual_mode) {
    return 'MANUAL';
  }
  if (context.auto_mode) {
    return 'AUTO';
  }
  return state === 'IDLE' ? 'IDLE' : '--';
}

function isEstop(state) {
  return state === 'ESTOP';
}

function isError(state) {
  return state === 'ERROR';
}

function hasActiveTask(context) {
  return context.has_active_task === true || context.outlet === 'OUTLET_1' || context.outlet === 'OUTLET_2';
}

function appendLog(text, kind = '') {
  const log = $('eventLog');
  if (!log) {
    return;
  }
  const item = document.createElement('div');
  item.className = `log-item ${kind}`;
  const time = new Date().toLocaleTimeString();
  item.textContent = `[${time}] ${text}`;
  log.prepend(item);
  while (log.children.length > 160) {
    log.removeChild(log.lastChild);
  }
}

function initStateSelect() {
  const select = $('targetStateSelect');
  if (!select) {
    return;
  }
  select.innerHTML = '';
  flowStates.forEach((state) => {
    const option = document.createElement('option');
    option.value = state;
    option.textContent = `${state} - ${flowLabels[state] || ''}`;
    if (!safeStateRequestTargets.has(state)) {
      option.dataset.risky = 'true';
    }
    select.appendChild(option);
  });
}

function renderFlow(activeState) {
  const list = $('flowList');
  if (!list) {
    return;
  }
  list.innerHTML = '';
  const activeIndex = flowStates.indexOf(activeState);
  flowStates.forEach((state, index) => {
    const item = document.createElement('div');
    const status = state === activeState ? ' active' : (activeIndex > index && activeIndex >= 0 ? ' complete' : '');
    item.className = `flow-item${status}`;

    const idx = document.createElement('span');
    idx.className = 'flow-index';
    idx.textContent = String(index + 1);

    const name = document.createElement('span');
    name.className = 'flow-name';
    name.textContent = state;

    const label = document.createElement('span');
    label.className = 'flow-cn';
    label.textContent = flowLabels[state] || '--';

    item.appendChild(idx);
    item.appendChild(name);
    item.appendChild(label);
    list.appendChild(item);
  });
}

function renderJoints(jointState) {
  const list = $('jointList');
  if (!list) {
    return;
  }
  list.innerHTML = '';

  const names = jointState.names || [];
  const positions = jointState.positions || [];
  if (!names.length) {
    const empty = document.createElement('div');
    empty.className = 'muted';
    empty.textContent = 'no joint state';
    list.appendChild(empty);
    return;
  }

  names.slice(0, 8).forEach((name, index) => {
    const pos = Number(positions[index] || 0);
    const row = document.createElement('div');
    row.className = 'joint-row';

    const left = document.createElement('div');
    const title = document.createElement('div');
    title.className = 'joint-name';
    title.textContent = name;
    const bar = document.createElement('div');
    bar.className = 'bar';
    const fill = document.createElement('span');
    const normalized = Math.max(0, Math.min(100, 50 + pos * 20));
    fill.style.width = `${normalized}%`;
    bar.appendChild(fill);
    left.appendChild(title);
    left.appendChild(bar);

    const right = document.createElement('div');
    right.textContent = pos.toFixed(3);

    row.appendChild(left);
    row.appendChild(right);
    list.appendChild(row);
  });
}

function commandRule(command, services, state, context) {
  if (!services || !services[command] || !services[command].ready) {
    return {enabled: false, reason: 'ROS 服务未就绪'};
  }

  if (command === 'simulate_estop') {
    return {enabled: true, reason: '立即触发急停'};
  }

  if (isEstop(state)) {
    if (command === 'clear_estop') {
      return {enabled: true, reason: '清除软件急停状态'};
    }
    if (command === 'request_reset') {
      return estopClearAcknowledged
        ? {enabled: true, reason: '急停已清除，可以复位'}
        : {enabled: false, reason: '先释放硬件急停并点击清除急停'};
    }
    return {enabled: false, reason: 'ESTOP 中禁止其他动作'};
  }

  if (isError(state)) {
    return command === 'request_reset'
      ? {enabled: true, reason: '错误状态允许人工复位'}
      : {enabled: false, reason: 'ERROR 中只允许复位'};
  }

  if (command === 'clear_estop') {
    return {enabled: false, reason: '当前未处于急停'};
  }

  if (command === 'request_reset') {
    return {enabled: false, reason: '当前无需复位'};
  }

  if (command === 'manual_mode') {
    return {enabled: state !== 'PAUSED', reason: state === 'PAUSED' ? '已经暂停' : '切换到人工暂停'};
  }

  if (command === 'auto_mode') {
    return {enabled: ['IDLE', 'WAIT_DISCHARGE', 'PAUSED'].includes(state), reason: '启动或恢复自动'};
  }

  if (command === 'step_once') {
    return {enabled: state === 'PAUSED', reason: state === 'PAUSED' ? '人工单步推进' : '需要先暂停到 PAUSED'};
  }

  if (command === 'outlet_1_done' || command === 'outlet_2_done' ||
      command === 'actual_cycle_outlet_1' || command === 'actual_cycle_outlet_2') {
    return {enabled: ['IDLE', 'WAIT_DISCHARGE', 'PAUSED'].includes(state), reason: '出料完成信号/真实循环触发'};
  }

  if (command === 'detection_done') {
    return {
      enabled: ['WAIT_DETECTION_DONE', 'PAUSED'].includes(state),
      reason: state === 'WAIT_DETECTION_DONE' ? '人工确认光谱检测完成' : '需要处于等待检测完成状态',
    };
  }

  if (workflowCommands.has(command)) {
    const busy = actionStates.has(state) || hasActiveTask(context);
    return {
      enabled: ['IDLE', 'PAUSED'].includes(state) && !busy,
      reason: busy ? '当前有任务或动作正在执行' : '手动调试动作',
    };
  }

  return {enabled: true, reason: ''};
}

function renderServices(services, state, context) {
  const values = Object.values(services || {});
  const ready = values.filter((item) => item.ready).length;
  setText('serviceBadge', values.length ? `服务 ${ready}/${values.length}` : '服务 --');
  const serviceBadge = $('serviceBadge');
  if (serviceBadge) {
    serviceBadge.className = values.length && ready === values.length ? 'badge badge-ok' : 'badge badge-warn';
  }

  document.querySelectorAll('button[data-command]').forEach((button) => {
    const command = button.dataset.command;
    const rule = commandRule(command, services, state, context);
    button.disabled = !rule.enabled || button.classList.contains('loading');
    button.title = rule.reason || button.title || '';
    button.setAttribute('aria-disabled', String(button.disabled));
  });
}

function renderStateRequestControl(state) {
  const button = $('publishStateRequest');
  const select = $('targetStateSelect');
  if (!button || !select) {
    return;
  }
  const target = select.value;
  let enabled = true;
  let reason = '提交状态机请求';

  if (state === 'ESTOP' && !['RESET', 'ESTOP'].includes(target)) {
    enabled = false;
    reason = 'ESTOP 中只允许请求 RESET 或保持 ESTOP';
  } else if (actionStates.has(target)) {
    enabled = false;
    reason = '动作状态只能由状态机流程进入，不能从 HMI 直接请求';
  } else if (!safeStateRequestTargets.has(target)) {
    enabled = false;
    reason = '该目标未开放为 HMI 手动请求';
  }

  button.disabled = !enabled || button.classList.contains('loading');
  button.title = reason;
}

function selectedPoseTunerTarget() {
  const select = $('poseTunerTargetSelect');
  if (!select || !select.value) {
    return null;
  }
  return poseTunerTargets.find((target) => target.name === select.value) || null;
}

function poseTunerSafetyRule(services, state, context, dryRun) {
  if (!poseTunerDebugMode) {
    return {enabled: false, reason: '调试模式关闭'};
  }
  if (!services || !services.pose_tuner_run || !services.pose_tuner_run.ready) {
    return {enabled: false, reason: 'pose_tuner 服务未就绪'};
  }
  if (!selectedPoseTunerTarget()) {
    return {enabled: false, reason: '未选择点位'};
  }
  if (state === 'ESTOP' || state === 'ERROR') {
    return {enabled: false, reason: `${state} 状态禁止点位调试`};
  }
  if (context.active_action_name || context.active_command_id) {
    return {enabled: false, reason: '状态机动作执行中'};
  }
  if (!dryRun) {
    const activeTask = hasActiveTask(context);
    const autoMode = context.auto_mode === true;
    if (autoMode || activeTask || !['IDLE', 'PAUSED', 'WAIT_DISCHARGE'].includes(state)) {
      return {enabled: false, reason: '真实执行要求空闲/人工安全状态'};
    }
  }
  return {enabled: true, reason: dryRun ? '只规划不运动' : '调试模式真实运动'};
}

function formatPoseTunerTarget(target) {
  if (!target) {
    return '未选择点位';
  }
  if (target.kind === 'joint') {
    return [
      target.name,
      `joint rad: ${(target.joints_rad || []).map((v) => Number(v).toFixed(3)).join(', ') || '--'}`,
      target.description || '',
    ].join('\n');
  }
  const xyz = target.xyz_mm || [];
  const rpy = target.rpy_deg || [];
  return [
    target.name,
    `XYZ mm: ${xyz.map((v) => Number(v).toFixed(2)).join(', ') || '--'}`,
    `RPY deg: ${rpy.map((v) => Number(v).toFixed(2)).join(', ') || '--'}`,
    target.description || '',
  ].join('\n');
}

function populatePoseTunerTargets(targets) {
  poseTunerTargets = Array.isArray(targets) ? targets : [];
  const select = $('poseTunerTargetSelect');
  if (!select) {
    return;
  }
  const current = select.value;
  select.innerHTML = '';
  if (!poseTunerTargets.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = poseTunerDebugMode ? '未读取到点位' : '请先开启调试模式';
    select.appendChild(option);
    return;
  }
  poseTunerTargets.forEach((target) => {
    const option = document.createElement('option');
    option.value = target.name;
    option.textContent = target.kind === 'joint' ? `${target.name} [joint]` : target.name;
    select.appendChild(option);
  });
  if (current && poseTunerTargets.some((target) => target.name === current)) {
    select.value = current;
  }
}

function renderPoseTuner(snapshot, state, context) {
  const services = snapshot.services || {};
  const poseTuner = snapshot.pose_tuner || {};
  if (!poseTunerTargets.length && Array.isArray(poseTuner.targets) && poseTuner.targets.length) {
    populatePoseTunerTargets(poseTuner.targets);
  }
  const panel = document.querySelector('.pose-tuner-panel');
  const mode = $('poseTunerDebugMode');
  const modeText = $('poseTunerModeText');
  const select = $('poseTunerTargetSelect');
  const dryRunButton = $('poseTunerDryRun');
  const executeButton = $('poseTunerExecute');
  const reloadButton = $('poseTunerReload');
  const stopButton = $('poseTunerStop');
  const targetCard = $('poseTunerTargetCard');
  const resultBox = $('poseTunerResult');
  const logBox = $('poseTunerLog');

  if (mode) {
    mode.checked = poseTunerDebugMode;
  }
  if (modeText) {
    modeText.textContent = poseTunerDebugMode ? '开启' : '关闭';
    modeText.style.color = poseTunerDebugMode ? '#b45309' : '';
  }
  if (panel) {
    panel.classList.toggle('debug-on', poseTunerDebugMode);
  }
  if (select) {
    select.disabled = !poseTunerDebugMode || poseTunerBusy;
  }
  if (reloadButton) {
    reloadButton.disabled = !poseTunerDebugMode || poseTunerBusy;
  }
  if (stopButton) {
    stopButton.disabled = !(services.pose_tuner_stop && services.pose_tuner_stop.ready) || poseTunerBusy;
    stopButton.title = stopButton.disabled ? 'pose_tuner stop 服务未就绪' : '停止当前 MoveIt 执行';
  }

  const dryRule = poseTunerSafetyRule(services, state, context, true);
  const executeRule = poseTunerSafetyRule(services, state, context, false);
  if (dryRunButton) {
    dryRunButton.disabled = !dryRule.enabled || poseTunerBusy;
    dryRunButton.title = dryRule.reason;
  }
  if (executeButton) {
    executeButton.disabled = !executeRule.enabled || poseTunerBusy;
    executeButton.title = executeRule.reason;
  }
  if (targetCard) {
    targetCard.textContent = formatPoseTunerTarget(selectedPoseTunerTarget());
  }

  if (poseTuner.last_result && resultBox && !poseTunerBusy) {
    resultBox.textContent = `${poseTuner.last_result.success ? 'OK' : 'FAIL'} | ${poseTuner.last_result.message || '--'}`;
  }
  if (logBox) {
    logBox.innerHTML = '';
    (poseTuner.log || []).slice(0, 8).forEach((entry) => {
      const item = document.createElement('div');
      item.textContent = `${entry.timestamp || '--'} ${entry.dry_run ? 'DRY' : 'RUN'} ${entry.target_name || '--'} ${entry.success ? 'OK' : 'FAIL'} ${entry.message || ''}`;
      logBox.appendChild(item);
    });
  }
}

function setPointResult(text, ok = true) {
  const box = $('pointConfigResult');
  if (!box) {
    return;
  }
  box.textContent = text || '';
  box.classList.toggle('failed', !ok);
  box.classList.toggle('success', ok && !!text);
}

function numericInputValue(id) {
  const el = $(id);
  const value = el ? Number(el.value) : NaN;
  if (!Number.isFinite(value)) {
    throw new Error(`${id} 不是有效数字`);
  }
  return value;
}

function setPointPoseInputs(name) {
  const pose = pointConfig && pointConfig.named_poses ? pointConfig.named_poses[name] : null;
  pointConfigSelectedPose = name || '';
  setText('pointPoseTitle', name || '未选择点位');
  const values = pose || {xyz: ['', '', ''], rpy: ['', '', '']};
  const ids = ['pointX', 'pointY', 'pointZ', 'pointRoll', 'pointPitch', 'pointYaw'];
  const data = [...(values.xyz || []), ...(values.rpy || [])];
  ids.forEach((id, index) => {
    const input = $(id);
    if (input) {
      input.value = data[index] === undefined || data[index] === null ? '' : String(data[index]);
      input.disabled = !pose;
    }
  });
}

function syncSelectedPointPoseFromInputs() {
  if (!pointConfig || !pointConfigSelectedPose) {
    return;
  }
  pointConfig.named_poses[pointConfigSelectedPose] = {
    xyz: [
      numericInputValue('pointX'),
      numericInputValue('pointY'),
      numericInputValue('pointZ'),
    ],
    rpy: [
      numericInputValue('pointRoll'),
      numericInputValue('pointPitch'),
      numericInputValue('pointYaw'),
    ],
  };
}

function createPointParamInput(section, key, value, labelText) {
  const label = document.createElement('label');
  label.className = 'point-param-row';
  const title = document.createElement('span');
  title.textContent = labelText || key;
  label.appendChild(title);

  if (typeof value === 'boolean') {
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.checked = value;
    input.dataset.pointSection = section;
    input.dataset.pointKey = key;
    input.addEventListener('change', onPointParamChanged);
    label.appendChild(input);
    return label;
  }

  if (Array.isArray(value)) {
    const group = document.createElement('div');
    group.className = 'point-param-array';
    value.forEach((item, index) => {
      const input = document.createElement('input');
      input.type = 'number';
      input.step = '0.001';
      input.value = String(item);
      input.dataset.pointSection = section;
      input.dataset.pointKey = key;
      input.dataset.pointIndex = String(index);
      input.addEventListener('input', onPointParamChanged);
      group.appendChild(input);
    });
    label.appendChild(group);
    return label;
  }

  if (key.endsWith('_pose')) {
    const select = document.createElement('select');
    select.dataset.pointSection = section;
    select.dataset.pointKey = key;
    Object.keys(pointConfig.named_poses || {}).forEach((name) => {
      const option = document.createElement('option');
      option.value = name;
      option.textContent = name;
      select.appendChild(option);
    });
    select.value = value === undefined || value === null ? '' : String(value);
    select.addEventListener('change', onPointParamChanged);
    label.appendChild(select);
    return label;
  }

  if (section === 'cleaning' && key === 'pour_direction') {
    const select = document.createElement('select');
    [
      {value: '-1', text: '负向 (-1)'},
      {value: '0', text: '自动 (0)'},
      {value: '1', text: '正向 (+1)'},
    ].forEach((item) => {
      const option = document.createElement('option');
      option.value = item.value;
      option.textContent = item.text;
      select.appendChild(option);
    });
    select.value = String(value);
    select.dataset.pointSection = section;
    select.dataset.pointKey = key;
    select.addEventListener('change', onPointParamChanged);
    label.appendChild(select);
    return label;
  }

  const input = document.createElement('input');
  input.type = key === 'axis' ? 'text' : 'number';
  input.step = key.includes('count') || key.includes('index') ? '1' : '0.001';
  input.value = value === undefined || value === null ? '' : String(value);
  input.dataset.pointSection = section;
  input.dataset.pointKey = key;
  input.addEventListener('input', onPointParamChanged);
  label.appendChild(input);
  return label;
}

function renderPointParamGroup(container, title, section, values, labels) {
  const group = document.createElement('div');
  group.className = 'point-param-group';
  const heading = document.createElement('h3');
  heading.textContent = title;
  group.appendChild(heading);
  Object.keys(values || {}).forEach((key) => {
    group.appendChild(createPointParamInput(section, key, values[key], labels[key] || key));
  });
  container.appendChild(group);
}

function renderPointConfig() {
  const select = $('pointPoseSelect');
  const editor = $('pointParamEditor');
  if (!pointConfig || !select || !editor) {
    return;
  }

  const current = pointConfigSelectedPose || select.value;
  select.innerHTML = '';
  Object.keys(pointConfig.named_poses || {}).forEach((name) => {
    const option = document.createElement('option');
    option.value = name;
    option.textContent = name;
    select.appendChild(option);
  });
  if (current && pointConfig.named_poses && pointConfig.named_poses[current]) {
    select.value = current;
  } else if (select.options.length) {
    select.value = select.options[0].value;
  }
  setPointPoseInputs(select.value);

  editor.innerHTML = '';
  renderPointParamGroup(editor, 'motion', 'motion', pointConfig.motion || {}, pointMotionLabels);
  renderPointParamGroup(editor, 'spectrometer_axis', 'spectrometer_axis', pointConfig.spectrometer_axis || {}, pointAxisLabels);
  renderPointParamGroup(editor, 'cleaning', 'cleaning', pointConfig.cleaning || {}, pointCleaningLabels);
}

function onPointPoseInputChanged() {
  try {
    syncSelectedPointPoseFromInputs();
    pointConfigDirty = true;
    setPointResult('点位已修改，保存后可点击“重载生效”。', true);
  } catch (error) {
    setPointResult(String(error), false);
  }
}

function onPointParamChanged(event) {
  if (!pointConfig) {
    return;
  }
  const input = event.target;
  const section = input.dataset.pointSection;
  const key = input.dataset.pointKey;
  if (!section || !key || !pointConfig[section]) {
    return;
  }
  try {
    if (input.dataset.pointIndex !== undefined) {
      const index = Number(input.dataset.pointIndex);
      const current = Array.isArray(pointConfig[section][key]) ? pointConfig[section][key] : [];
      current[index] = Number(input.value);
      if (!Number.isFinite(current[index])) {
        throw new Error(`${section}.${key}[${index}] 不是有效数字`);
      }
      pointConfig[section][key] = current;
    } else if (input.type === 'checkbox') {
      pointConfig[section][key] = input.checked;
    } else if (input.tagName === 'SELECT') {
      const value = input.value.trim();
      if (!value) {
        throw new Error(`${section}.${key} 不能为空`);
      }
      pointConfig[section][key] = value;
    } else if (key === 'axis') {
      const axis = input.value.trim().toLowerCase();
      if (!['x', 'y', 'z'].includes(axis)) {
        throw new Error('axis 只能是 x/y/z');
      }
      pointConfig[section][key] = axis;
    } else {
      const value = Number(input.value);
      if (!Number.isFinite(value)) {
        throw new Error(`${section}.${key} 不是有效数字`);
      }
      pointConfig[section][key] = value;
    }
    pointConfigDirty = true;
    setPointResult('参数已修改，保存后可点击“重载生效”。', true);
  } catch (error) {
    setPointResult(String(error), false);
  }
}

async function loadPointConfig() {
  const saveButton = $('pointConfigSave');
  const reloadButton = $('pointConfigReload');
  const applyReloadButton = $('pointConfigApplyReload');
  [saveButton, reloadButton, applyReloadButton].forEach((button) => {
    if (button) {
      button.disabled = true;
    }
  });
  setPointResult('读取点位配置...', true);
  try {
    const response = await fetch('/api/point_config');
    const result = await response.json();
    if (!result.success) {
      throw new Error(result.message || '读取失败');
    }
    pointConfig = result.config || {};
    pointConfigDirty = false;
    setText('pointConfigPath', result.path || '--');
    renderPointConfig();
    setPointResult('点位配置已读取。', true);
    appendLog(`point_config load: ${result.path || '--'}`, 'log-ok');
  } catch (error) {
    setPointResult(`读取点位配置失败: ${error}`, false);
    appendLog(`point_config load failed: ${error}`, 'log-error');
  } finally {
    [saveButton, reloadButton, applyReloadButton].forEach((button) => {
      if (button) {
        button.disabled = false;
      }
    });
  }
}

async function savePointConfig(button) {
  if (!pointConfig) {
    setPointResult('请先读取点位配置。', false);
    return;
  }
  try {
    syncSelectedPointPoseFromInputs();
  } catch (error) {
    setPointResult(String(error), false);
    return;
  }

  const confirmed = await confirmAction(
    '保存会覆盖 spectrometer_cell.yaml 中的点位和路径参数，并自动生成备份。保存后可点击“重载生效”。确认保存？',
    '保存点位配置');
  if (!confirmed) {
    appendLog('CANCEL point_config save', 'log-warn');
    return;
  }

  setButtonFeedback(button, 'loading');
  setPointResult('保存点位配置...', true);
  try {
    const response = await fetch('/api/point_config', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({config: pointConfig}),
    });
    const result = await response.json();
    if (!result.success) {
      throw new Error(result.message || '保存失败');
    }
    pointConfigDirty = false;
    const reloadResult = result.reload_result || {};
    const reloadText = result.runtime_reloaded
      ? '运行节点已自动重载'
      : `运行节点未重载: ${reloadResult.message || '请空闲后点击“重载生效”'}`;
    setPointResult(`${result.message || '保存成功'}；备份: ${result.backup_path || '--'}；${reloadText}`, !!result.runtime_reloaded);
    appendLog(`point_config save: ${(result.changed || []).length} fields`, 'log-ok');
    if (result.reload_result) {
      appendLog(`point_config auto reload: ${result.reload_result.message || '--'}`, result.runtime_reloaded ? 'log-ok' : 'log-warn');
    }
    setButtonFeedback(button, 'success');
  } catch (error) {
    setPointResult(`保存点位配置失败: ${error}`, false);
    appendLog(`point_config save failed: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    if (button) {
      button.classList.remove('loading');
    }
  }
}

async function reloadPointConfigRuntime(button) {
  setButtonFeedback(button, 'loading');
  setPointResult('正在通知运行节点重载配置...', true);
  try {
    const response = await fetch('/api/point_config/reload', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({source: 'web_hmi'}),
    });
    const result = await response.json();
    if (!result.success) {
      throw new Error(result.message || '重载失败');
    }
    setPointResult(result.message || '运行节点已重载配置', true);
    appendLog(`point_config reload: ${result.message || '--'}`, 'log-ok');
    setButtonFeedback(button, 'success');
  } catch (error) {
    setPointResult(`重载配置失败: ${error}`, false);
    appendLog(`point_config reload failed: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    if (button) {
      button.classList.remove('loading');
    }
  }
}

function cameraHealthClass(channel) {
  if (!channel || !channel.available) {
    return 'pill pill-warn';
  }
  if (channel.age_sec !== null && channel.age_sec !== undefined && channel.age_sec > 2.5) {
    return 'pill pill-error';
  }
  return 'pill pill-ok';
}

function cameraHealthText(channel) {
  if (!channel || !channel.available) {
    return '无图像';
  }
  if (channel.age_sec !== null && channel.age_sec !== undefined && channel.age_sec > 2.5) {
    return '图像延迟';
  }
  return '图像正常';
}

function refreshCameraImage(channel) {
  const img = $('cameraImage');
  if (!img) {
    return;
  }

  const baseUrl = cameraMode === 'rgb'
    ? '/api/camera/rgb_stream.mjpg'
    : (channel && channel.url ? channel.url : `/api/camera/${cameraMode}`);
  const now = Date.now();
  if (cameraMode === 'rgb') {
    if (img.dataset.baseUrl !== baseUrl) {
      img.dataset.baseUrl = baseUrl;
      img.src = `${baseUrl}?t=${now}`;
      lastCameraRefreshMs = now;
    }
    return;
  }

  if (img.dataset.baseUrl === baseUrl && now - lastCameraRefreshMs < CAMERA_REFRESH_INTERVAL_MS) {
    return;
  }
  img.dataset.baseUrl = baseUrl;
  img.src = `${baseUrl}?t=${now}`;
  lastCameraRefreshMs = now;
}

function renderCamera(camera) {
  const channel = camera && camera[cameraMode] ? camera[cameraMode] : {};
  const health = $('cameraHealth');
  if (health) {
    health.textContent = cameraHealthText(channel);
    health.className = cameraHealthClass(channel);
  }

  const size = channel.width && channel.height ? `${channel.width}x${channel.height}` : '--';
  const encoding = channel.encoding || '--';
  setText('cameraInfo', `${channel.topic || '--'} | ${encoding} | ${size}`);
  setText('cameraAge', formatAge(channel.age_sec));
  refreshCameraImage(channel);

  document.querySelectorAll('[data-camera-mode]').forEach((button) => {
    button.classList.toggle('active', button.dataset.cameraMode === cameraMode);
  });
}

function renderMotion(motion, services) {
  const slider = $('speedScaleSlider');
  const value = $('speedScaleValue');
  const applyButton = $('applySpeedScale');
  const resetButton = $('resetSpeedScale');
  const restartCameraButton = $('restartCameraButton');
  const pointConfigApplyReloadButton = $('pointConfigApplyReload');
  const percent = Number(motion && motion.speed_percent ? motion.speed_percent : speedScalePercent);
  const nowMs = Date.now();
  if (speedScalePendingPercent !== null && Number.isFinite(percent) &&
      Math.round(percent) === Math.round(speedScalePendingPercent)) {
    speedScalePendingPercent = null;
    speedScalePendingUntilMs = 0;
  } else if (speedScalePendingPercent !== null && nowMs >= speedScalePendingUntilMs) {
    speedScalePendingPercent = null;
    speedScalePendingUntilMs = 0;
  }
  const hasPendingSpeed =
    speedScalePendingPercent !== null && nowMs < speedScalePendingUntilMs;

  const sliderActive = slider && document.activeElement === slider;
  if (!speedScaleDirty && !sliderActive && !hasPendingSpeed && Number.isFinite(percent)) {
    speedScalePercent = Math.max(20, Math.min(120, Math.round(percent)));
  }
  if (slider && !speedScaleDirty && !sliderActive) {
    slider.value = String(speedScalePercent);
  }
  if (value) {
    const displayPercent = slider ? Number(slider.value) : speedScalePercent;
    value.textContent = `${displayPercent}%`;
  }

  const ready = !!(services && services.speed_scale && services.speed_scale.ready);
  [applyButton, resetButton].forEach((button) => {
    if (button) {
      button.disabled = !ready || button.classList.contains('loading');
      button.title = ready ? '设置当前动作速度比例' : '速度控制服务未就绪';
    }
  });

  if (restartCameraButton) {
    const cameraReady = !!(services && services.camera_restart && services.camera_restart.ready);
    restartCameraButton.disabled = !cameraReady || restartCameraButton.classList.contains('loading');
    restartCameraButton.title = cameraReady ? '重启 Gemini305 相机驱动' : '相机重启脚本未就绪';
  }

  if (pointConfigApplyReloadButton) {
    const reloadReady = !!(services && services.reload_config && services.reload_config.ready);
    pointConfigApplyReloadButton.disabled =
      !reloadReady || pointConfigApplyReloadButton.classList.contains('loading');
    pointConfigApplyReloadButton.title = reloadReady
      ? '让 spectrometer_cell 重新读取已保存的点位配置'
      : '配置重载服务未就绪';
  }
}

function renderSignal(signal) {
  const name = signal.name || '--';
  const active = signal.active === true ? 'active' : signal.active === false ? 'inactive' : '--';
  const workflow = signal.workflow_name || '--';
  setText('recentEventName', name);
  setText('recentEventDetail', `${signal.source || '--'} | code=${signal.code ?? '--'} | ${active} | ${workflow} | ${formatAge(signal.age_sec)}`);
}

function renderLaser(snapshot, context) {
  const laser = snapshot.laser || {};
  const raw = laser.raw_distance_mm ?? laser.distance_raw_mm ?? laser.distance_mm;
  const filtered = laser.filtered_distance_mm ?? laser.distance_filtered_mm ?? laser.distance_mm;
  const axis = laser.axis_position_mm ?? laser.spectrometer_axis_mm ?? laser.position_mm;
  const laserAgeText = formatAge(laser.age_sec);

  setText('laserDistance', laser.distance_mm === null || laser.distance_mm === undefined ? '--' : `${Number(laser.distance_mm).toFixed(1)} mm`);
  setText('laserRaw', formatMm(raw));
  setText('laserFiltered', formatMm(filtered));
  setText('laserAxis', formatMm(axis));
  setText('laserStatus', `${laser.valid ? 'valid' : 'invalid'} | ${laser.status || 'no status'} | ${laser.source || '--'}`);
  setText('laserAge', laser.age_sec !== null && laser.age_sec !== undefined && laser.age_sec > 2.0 ? `${laserAgeText} 延迟` : laserAgeText);
  setText('placeLaser', formatMm(context.spectrometer_position_before_place_mm));
  setText('pickLaser', formatMm(context.spectrometer_position_before_pick_mm));
}

function renderToolPose(snapshot) {
  const pose = snapshot.tool_pose || {};
  const pos = pose.position_mm || {};
  const quat = pose.quaternion_xyzw || {};
  const rpy = pose.rpy_deg || {};
  const available = pose.available === true;

  setText('toolPoseAge', formatAge(pose.age_sec));
  setText('toolPoseFrames', `${pose.base_frame || 'base_link'} -> ${pose.tool_frame || 'gripper_center'}`);
  const poseStatusText = pose.status === 'filtered_spike'
    ? `TF FILTERED ${pose.filter_reject_count || 0}`
    : (available ? 'TF OK' : (pose.status || 'no tf'));
  setText('toolPoseStatus', poseStatusText);
  setText('toolPoseX', formatPoseMm(pos.x));
  setText('toolPoseY', formatPoseMm(pos.y));
  setText('toolPoseZ', formatPoseMm(pos.z));
  setText('toolQuatX', formatQuat(quat.x));
  setText('toolQuatY', formatQuat(quat.y));
  setText('toolQuatZ', formatQuat(quat.z));
  setText('toolQuatW', formatQuat(quat.w));
  setText('toolPoseRpy', `R ${formatDeg(rpy.roll)} / P ${formatDeg(rpy.pitch)} / Y ${formatDeg(rpy.yaw)}`);

  const status = $('toolPoseStatus');
  if (status) {
    status.className = available && pose.status !== 'filtered_spike' ? 'pose-ok' : 'pose-bad';
  }
}

function render(snapshot) {
  latestSnapshot = snapshot;
  const cell = snapshot.spectrometer_cell || {};
  const context = cell.context || {};
  const state = cell.state || context.state || 'UNKNOWN';
  const mode = deriveMode(context, state);

  if (state !== 'ESTOP') {
    estopClearAcknowledged = false;
  }

  setText('stateText', state);
  setText('stateAge', formatAge(cell.state_age_sec));
  setText('stateDuration', `持续 ${formatAge(cell.state_age_sec)}`);
  setText('previousState', `上一步 ${context.previous_state || '--'}`);
  setText('transitionReason', `原因 ${context.last_transition_reason || '--'}`);
  setText('modeText', mode);
  setText('cycleId', context.cycle_id ?? '--');
  setText('outletId', context.outlet || '--');
  setText('pendingQueue', formatQueue(context.pending_outlets));
  setText('cupFlags', [
    `夹持:${boolText(context.cup_in_gripper)}`,
    `已清理:${boolText(context.cup_cleaned)}`,
    `已归还:${boolText(context.cup_returned)}`,
  ].join(' / '));
  setText('spectrometerFlags', [
    `占用:${boolText(context.spectrometer_occupied)}`,
    `检测启动:${boolText(context.detection_started)}`,
    `检测完成:${boolText(context.detection_done)}`,
  ].join(' / '));
  setText('activeCommand', context.active_command_id || context.active_action_name
    ? `${context.active_command_id || '--'} | ${context.active_action_name || '--'}`
    : '--');
  setText('outletStatus', formatQueue(context.outlet_status));
  setText('commandMode', mode);

  const stateBadge = $('stateBadge');
  if (stateBadge) {
    stateBadge.textContent = state;
    stateBadge.className = stateClass(state);
  }
  const modeBadge = $('modeBadge');
  if (modeBadge) {
    modeBadge.textContent = `模式 ${mode}`;
    modeBadge.className = stateClass(state);
  }
  const estopZone = $('estopZone');
  if (estopZone) {
    estopZone.classList.toggle('estop-active', state === 'ESTOP');
  }

  if (state !== lastState) {
    appendLog(`STATE ${lastState || '--'} -> ${state}`, state === 'ERROR' || state === 'ESTOP' ? 'log-error' : 'log-ok');
    lastState = state;
  }

  const error = cell.error || context.error || context.error_message || '';
  const errorBox = $('errorBox');
  if (errorBox) {
    if (error) {
      errorBox.textContent = error;
      errorBox.classList.remove('hidden');
    } else {
      errorBox.classList.add('hidden');
    }
  }

  renderFlow(state);
  renderCamera(snapshot.camera || {});
  renderLaser(snapshot, context);
  renderToolPose(snapshot);
  renderSignal(snapshot.external_signal || {});
  renderJoints(snapshot.joint_state || {});
  setText('jointAge', formatAge((snapshot.joint_state || {}).age_sec));
  renderServices(snapshot.services || {}, state, context);
  renderMotion(snapshot.motion || {}, snapshot.services || {});
  renderStateRequestControl(state);
  renderPoseTuner(snapshot, state, context);
}

async function fetchSnapshot() {
  const response = await fetch('/api/status');
  if (!response.ok) {
    throw new Error(`status ${response.status}`);
  }
  return response.json();
}

function connectEvents() {
  if (eventSource) {
    eventSource.close();
  }

  eventSource = new EventSource('/api/events');
  eventSource.onopen = () => setConnection(true);
  eventSource.onerror = () => setConnection(false);
  eventSource.onmessage = (event) => {
    setConnection(true);
    render(JSON.parse(event.data));
  };
}

function setButtonFeedback(button, state) {
  if (!button) {
    return;
  }
  button.classList.remove('loading', 'success', 'failed');
  if (state) {
    button.classList.add(state);
  }
  if (state === 'success' || state === 'failed') {
    window.setTimeout(() => button.classList.remove(state), 1400);
  }
}

function confirmAction(message, title = '确认操作') {
  if (!message) {
    return Promise.resolve(true);
  }
  const overlay = $('confirmOverlay');
  const ok = $('confirmOk');
  const cancel = $('confirmCancel');
  const msg = $('confirmMessage');
  const heading = $('confirmTitle');
  if (!overlay || !ok || !cancel || !msg || !heading) {
    return Promise.resolve(window.confirm(message));
  }

  heading.textContent = title;
  msg.textContent = message;
  overlay.classList.remove('hidden');

  return new Promise((resolve) => {
    const cleanup = (value) => {
      overlay.classList.add('hidden');
      ok.removeEventListener('click', onOk);
      cancel.removeEventListener('click', onCancel);
      overlay.removeEventListener('click', onOverlay);
      resolve(value);
    };
    const onOk = () => cleanup(true);
    const onCancel = () => cleanup(false);
    const onOverlay = (event) => {
      if (event.target === overlay) {
        cleanup(false);
      }
    };
    ok.addEventListener('click', onOk);
    cancel.addEventListener('click', onCancel);
    overlay.addEventListener('click', onOverlay);
  });
}

async function sendCommand(command, button) {
  const resultBox = $('commandResult');
  if (resultBox) {
    resultBox.textContent = '发送中...';
  }
  setButtonFeedback(button, 'loading');
  appendLog(`MANUAL COMMAND ${command}`, 'log-warn');

  try {
    const response = await fetch('/api/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({command}),
    });
    const result = await response.json();
    if (command === 'clear_estop' && result.success) {
      estopClearAcknowledged = true;
    }
    if (resultBox) {
      resultBox.textContent = result.message || (result.success ? 'ok' : 'failed');
    }
    appendLog(`${command}: ${result.message || '--'}`, result.success ? 'log-ok' : 'log-error');
    setButtonFeedback(button, result.success ? 'success' : 'failed');
  } catch (error) {
    if (resultBox) {
      resultBox.textContent = String(error);
    }
    appendLog(`${command}: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    if (button) {
      button.classList.remove('loading');
    }
    if (latestSnapshot) {
      renderServices(latestSnapshot.services || {}, (latestSnapshot.spectrometer_cell || {}).state || 'UNKNOWN', (latestSnapshot.spectrometer_cell || {}).context || {});
    }
  }
}

async function sendStateRequest(button) {
  const target = $('targetStateSelect') ? $('targetStateSelect').value : '';
  const reason = $('stateRequestReason') ? $('stateRequestReason').value.trim() : '';
  const force = $('forceStateRequest') ? $('forceStateRequest').checked : false;
  const resultBox = $('commandResult');

  if (!target) {
    if (resultBox) {
      resultBox.textContent = '请选择目标状态';
    }
    return;
  }
  if (!reason) {
    if (resultBox) {
      resultBox.textContent = '状态请求必须填写原因';
    }
    return;
  }

  setButtonFeedback(button, 'loading');
  appendLog(`STATE REQUEST target=${target} force=${force} reason=${reason}`, 'log-warn');
  try {
    const response = await fetch('/api/state_request', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({
        target_state: target,
        reason,
        force,
        source: 'web_hmi',
        requested_at: new Date().toISOString(),
      }),
    });
    const result = await response.json();
    if (resultBox) {
      resultBox.textContent = result.message || (result.success ? 'state request accepted' : 'state request rejected');
    }
    appendLog(`state_request ${target}: ${result.message || '--'}`, result.success ? 'log-ok' : 'log-error');
    setButtonFeedback(button, result.success ? 'success' : 'failed');
  } catch (error) {
    if (resultBox) {
      resultBox.textContent = String(error);
    }
    appendLog(`state_request ${target}: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    if (button) {
      button.classList.remove('loading');
    }
  }
}

async function sendSpeedScale(percent, button) {
  const resultBox = $('commandResult');
  const scale = Math.max(0.20, Math.min(1.20, Number(percent) / 100.0));
  setButtonFeedback(button, 'loading');
  appendLog(`SET SPEED ${Math.round(scale * 100)}%`, 'log-warn');

  try {
    const response = await fetch('/api/speed_scale', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({scale}),
    });
    const result = await response.json();
    if (result.success && Number.isFinite(Number(result.applied_percent))) {
      speedScalePercent = Number(result.applied_percent);
      speedScaleDirty = false;
      speedScalePendingPercent = speedScalePercent;
      speedScalePendingUntilMs = Date.now() + 5000;
      const slider = $('speedScaleSlider');
      if (slider) {
        slider.value = String(speedScalePercent);
      }
      setText('speedScaleValue', `${speedScalePercent}%`);
    }
    if (resultBox) {
      resultBox.textContent = result.message || (result.success ? 'speed set' : 'speed set failed');
    }
    appendLog(`speed_scale: ${result.message || '--'}`, result.success ? 'log-ok' : 'log-error');
    setButtonFeedback(button, result.success ? 'success' : 'failed');
  } catch (error) {
    if (resultBox) {
      resultBox.textContent = String(error);
    }
    appendLog(`speed_scale: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    if (button) {
      button.classList.remove('loading');
    }
  }
}

async function restartCamera(button) {
  const resultBox = $('commandResult');
  setButtonFeedback(button, 'loading');
  appendLog('CAMERA RESTART', 'log-warn');

  try {
    const response = await fetch('/api/camera/restart', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({source: 'web_hmi'}),
    });
    const result = await response.json();
    if (resultBox) {
      resultBox.textContent = result.message || (result.success ? 'camera restart started' : 'camera restart failed');
    }
    appendLog(`camera_restart: ${result.message || '--'}`, result.success ? 'log-ok' : 'log-error');
    setButtonFeedback(button, result.success ? 'success' : 'failed');
  } catch (error) {
    if (resultBox) {
      resultBox.textContent = String(error);
    }
    appendLog(`camera_restart: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    if (button) {
      button.classList.remove('loading');
    }
  }
}

async function loadPoseTunerTargets() {
  const resultBox = $('poseTunerResult');
  if (resultBox) {
    resultBox.textContent = '读取点位列表...';
  }
  poseTunerBusy = true;
  try {
    const response = await fetch('/api/pose_tuner/list');
    const result = await response.json();
    if (result.success) {
      populatePoseTunerTargets(result.targets || []);
      if (resultBox) {
        resultBox.textContent = result.message || '点位已刷新';
      }
      appendLog(`pose_tuner list: ${result.message || '--'}`, 'log-ok');
    } else {
      if (resultBox) {
        resultBox.textContent = result.message || '读取点位失败';
      }
      appendLog(`pose_tuner list failed: ${result.message || '--'}`, 'log-error');
    }
  } catch (error) {
    if (resultBox) {
      resultBox.textContent = String(error);
    }
    appendLog(`pose_tuner list error: ${error}`, 'log-error');
  } finally {
    poseTunerBusy = false;
    if (latestSnapshot) {
      renderPoseTuner(
        latestSnapshot,
        (latestSnapshot.spectrometer_cell || {}).state || 'UNKNOWN',
        (latestSnapshot.spectrometer_cell || {}).context || {});
    }
  }
}

async function runPoseTunerTarget(dryRun, button) {
  const target = selectedPoseTunerTarget();
  const resultBox = $('poseTunerResult');
  if (!target) {
    if (resultBox) {
      resultBox.textContent = '请先选择点位';
    }
    return;
  }

  if (!dryRun) {
    const confirmed = await confirmAction(
      [
        '当前处于点位调试模式。',
        `目标点位：${target.name}`,
        '该操作会真实移动机械臂。',
        '确认执行？',
      ].join('\n'),
      '点位真实执行确认');
    if (!confirmed) {
      appendLog(`CANCEL pose_tuner execute ${target.name}`, 'log-warn');
      return;
    }
  }

  poseTunerBusy = true;
  setButtonFeedback(button, 'loading');
  if (resultBox) {
    resultBox.textContent = dryRun ? '发送 dry_run 规划...' : '发送真实执行...';
  }
  appendLog(`POSE_TUNER ${dryRun ? 'DRY_RUN' : 'EXECUTE'} ${target.name}`, dryRun ? 'log-warn' : 'log-error');

  try {
    const response = await fetch('/api/pose_tuner/run', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({
        target_name: target.name,
        dry_run: dryRun,
        debug_mode: poseTunerDebugMode,
        source: 'web_hmi',
        requested_at: new Date().toISOString(),
      }),
    });
    const result = await response.json();
    if (resultBox) {
      resultBox.textContent = result.message || (result.success ? 'ok' : 'failed');
    }
    appendLog(`pose_tuner ${target.name}: ${result.message || '--'}`, result.success ? 'log-ok' : 'log-error');
    setButtonFeedback(button, result.success ? 'success' : 'failed');
  } catch (error) {
    if (resultBox) {
      resultBox.textContent = String(error);
    }
    appendLog(`pose_tuner ${target.name}: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    poseTunerBusy = false;
    if (button) {
      button.classList.remove('loading');
    }
    if (latestSnapshot) {
      renderPoseTuner(
        latestSnapshot,
        (latestSnapshot.spectrometer_cell || {}).state || 'UNKNOWN',
        (latestSnapshot.spectrometer_cell || {}).context || {});
    }
  }
}

async function stopPoseTuner(button) {
  const resultBox = $('poseTunerResult');
  poseTunerBusy = true;
  setButtonFeedback(button, 'loading');
  if (resultBox) {
    resultBox.textContent = '发送 MoveIt stop...';
  }
  try {
    const response = await fetch('/api/pose_tuner/stop', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({source: 'web_hmi', requested_at: new Date().toISOString()}),
    });
    const result = await response.json();
    if (resultBox) {
      resultBox.textContent = result.message || (result.success ? 'stopped' : 'stop failed');
    }
    appendLog(`pose_tuner stop: ${result.message || '--'}`, result.success ? 'log-ok' : 'log-error');
    setButtonFeedback(button, result.success ? 'success' : 'failed');
  } catch (error) {
    if (resultBox) {
      resultBox.textContent = String(error);
    }
    appendLog(`pose_tuner stop: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    poseTunerBusy = false;
    if (button) {
      button.classList.remove('loading');
    }
  }
}

function wireButtons() {
  document.querySelectorAll('[data-page-target]').forEach((button) => {
    button.addEventListener('click', () => setActivePage(button.dataset.pageTarget));
  });

  document.querySelectorAll('button[data-command]').forEach((button) => {
    button.addEventListener('click', async () => {
      if (button.disabled) {
        return;
      }
      const command = button.dataset.command;
      const message = command === 'simulate_estop' ? '' : button.dataset.confirm;
      if (message && !(await confirmAction(message))) {
        appendLog(`CANCEL ${command}`, 'log-warn');
        return;
      }
      sendCommand(command, button);
    });
  });

  const stateRequestButton = $('publishStateRequest');
  if (stateRequestButton) {
    stateRequestButton.addEventListener('click', async () => {
      if (stateRequestButton.disabled) {
        return;
      }
      const message = stateRequestButton.dataset.confirm;
      if (message && !(await confirmAction(message, '状态请求确认'))) {
        appendLog('CANCEL state_request', 'log-warn');
        return;
      }
      sendStateRequest(stateRequestButton);
    });
  }

  document.querySelectorAll('[data-camera-mode]').forEach((button) => {
    button.addEventListener('click', () => {
      cameraMode = button.dataset.cameraMode;
      lastCameraRefreshMs = 0;
      renderCamera(latestSnapshot ? latestSnapshot.camera || {} : {});
    });
  });

  const restartCameraButton = $('restartCameraButton');
  if (restartCameraButton) {
    restartCameraButton.addEventListener('click', () => {
      if (!restartCameraButton.disabled) {
        restartCamera(restartCameraButton);
      }
    });
  }

  const speedSlider = $('speedScaleSlider');
  if (speedSlider) {
    speedSlider.addEventListener('input', () => {
      speedScalePercent = Number(speedSlider.value);
      speedScaleDirty = true;
      setText('speedScaleValue', `${speedScalePercent}%`);
    });
  }

  const applySpeed = $('applySpeedScale');
  if (applySpeed) {
    applySpeed.addEventListener('click', () => {
      if (!applySpeed.disabled) {
        sendSpeedScale(speedScalePercent, applySpeed);
      }
    });
  }

  const resetSpeed = $('resetSpeedScale');
  if (resetSpeed) {
    resetSpeed.addEventListener('click', () => {
      speedScalePercent = 100;
      speedScaleDirty = true;
      if (speedSlider) {
        speedSlider.value = '100';
      }
      setText('speedScaleValue', '100%');
      if (!resetSpeed.disabled) {
        sendSpeedScale(100, resetSpeed);
      }
    });
  }

  const select = $('targetStateSelect');
  if (select) {
    select.addEventListener('change', () => {
      const cell = latestSnapshot ? latestSnapshot.spectrometer_cell || {} : {};
      renderStateRequestControl(cell.state || 'UNKNOWN');
    });
  }

  const poseMode = $('poseTunerDebugMode');
  if (poseMode) {
    poseMode.addEventListener('change', async () => {
      poseTunerDebugMode = poseMode.checked;
      appendLog(`POSE_TUNER_DEBUG ${poseTunerDebugMode ? 'ON' : 'OFF'}`, poseTunerDebugMode ? 'log-warn' : '');
      if (poseTunerDebugMode && !poseTunerTargets.length) {
        await loadPoseTunerTargets();
      } else if (latestSnapshot) {
        renderPoseTuner(
          latestSnapshot,
          (latestSnapshot.spectrometer_cell || {}).state || 'UNKNOWN',
          (latestSnapshot.spectrometer_cell || {}).context || {});
      }
    });
  }

  const poseSelect = $('poseTunerTargetSelect');
  if (poseSelect) {
    poseSelect.addEventListener('change', () => {
      if (latestSnapshot) {
        renderPoseTuner(
          latestSnapshot,
          (latestSnapshot.spectrometer_cell || {}).state || 'UNKNOWN',
          (latestSnapshot.spectrometer_cell || {}).context || {});
      }
    });
  }

  const poseReload = $('poseTunerReload');
  if (poseReload) {
    poseReload.addEventListener('click', () => {
      if (!poseReload.disabled) {
        loadPoseTunerTargets();
      }
    });
  }

  const poseDryRun = $('poseTunerDryRun');
  if (poseDryRun) {
    poseDryRun.addEventListener('click', () => {
      if (!poseDryRun.disabled) {
        runPoseTunerTarget(true, poseDryRun);
      }
    });
  }

  const poseExecute = $('poseTunerExecute');
  if (poseExecute) {
    poseExecute.addEventListener('click', () => {
      if (!poseExecute.disabled) {
        runPoseTunerTarget(false, poseExecute);
      }
    });
  }

  const poseStop = $('poseTunerStop');
  if (poseStop) {
    poseStop.addEventListener('click', () => {
      if (!poseStop.disabled) {
        stopPoseTuner(poseStop);
      }
    });
  }

  const pointPoseSelect = $('pointPoseSelect');
  if (pointPoseSelect) {
    pointPoseSelect.addEventListener('change', () => {
      try {
        syncSelectedPointPoseFromInputs();
      } catch (error) {
        setPointResult(String(error), false);
      }
      setPointPoseInputs(pointPoseSelect.value);
    });
  }

  ['pointX', 'pointY', 'pointZ', 'pointRoll', 'pointPitch', 'pointYaw'].forEach((id) => {
    const input = $(id);
    if (input) {
      input.addEventListener('input', onPointPoseInputChanged);
    }
  });

  const pointReload = $('pointConfigReload');
  if (pointReload) {
    pointReload.addEventListener('click', loadPointConfig);
  }

  const pointSave = $('pointConfigSave');
  if (pointSave) {
    pointSave.addEventListener('click', () => savePointConfig(pointSave));
  }

  const pointApplyReload = $('pointConfigApplyReload');
  if (pointApplyReload) {
    pointApplyReload.addEventListener('click', () => {
      if (!pointApplyReload.disabled) {
        reloadPointConfigRuntime(pointApplyReload);
      }
    });
  }
}

async function pollFallback() {
  try {
    render(await fetchSnapshot());
    setConnection(true);
  } catch (error) {
    setConnection(false);
  }
}

async function boot() {
  initStateSelect();
  setActivePage(activePage);
  renderFlow('UNKNOWN');
  renderCamera({});
  renderMotion({}, {});
  wireButtons();
  connectEvents();

  try {
    render(await fetchSnapshot());
    setConnection(true);
  } catch (error) {
    setConnection(false);
    appendLog(`status fetch failed: ${error}`, 'log-error');
  }

  setInterval(() => {
    if (latestSnapshot) {
      renderCamera(latestSnapshot.camera || {});
    }
  }, 1000);

  setInterval(() => {
    if (!latestSnapshot) {
      return;
    }
    const camera = latestSnapshot.camera || {};
    const channel = camera && camera[cameraMode] ? camera[cameraMode] : {};
    refreshCameraImage(channel);
  }, CAMERA_REFRESH_INTERVAL_MS);

  setInterval(() => {
    if (!eventSource || eventSource.readyState === EventSource.CLOSED) {
      pollFallback();
    }
  }, 2500);
}

boot();
