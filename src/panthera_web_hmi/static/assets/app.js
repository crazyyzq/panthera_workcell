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
  STOPPED: '工作站已停止',
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

const modeLabels = {
  OFFLINE: '未启动',
  AUTO: '自动',
  MANUAL: '人工',
  PAUSED: '已暂停',
  IDLE: '空闲',
  ESTOP: '急停',
  ERROR: '故障',
};

const debugPhaseLabels = {
  inactive: '未进入调试',
  pausing: '正在暂停自动流程',
  recovering_home: '正在确认 Home',
  moving_safe: '正在前往安全点',
  moving_target: '正在前往目标点',
  ready: '可以点动',
  jogging: '正在点动',
  saving: '正在保存并校验',
  returning: '正在安全退出',
  error: '调试异常',
};

const coreServiceNames = [
  'auto_mode',
  'manual_mode',
  'actual_cycle_outlet_1',
  'actual_cycle_outlet_2',
  'detection_done',
  'speed_scale',
  'spectrometer_stop_motion',
  'spectrometer_recover_home',
  'reload_config',
];

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

const CAMERA_REFRESH_INTERVAL_MS = 33;

let lastState = null;
let eventSource = null;
let latestSnapshot = null;
let cameraMode = 'rgb';
let lastCameraRefreshMs = 0;
let estopClearAcknowledged = false;
let speedScalePercent = 100;
let speedScaleDirty = false;
let controlModeDirty = false;
let speedScalePendingPercent = null;
let speedScalePendingUntilMs = 0;
let motionCatalog = null;
let motionCatalogReferences = {};
let motionCatalogSelectedPoint = '';
let motionCatalogSelectedRoute = '';
let motionCatalogDirty = false;
let motionCatalogShowAdvancedPoints = false;
let debugBusy = false;
let debugTargetPoint = '';
let activePage = localStorage.getItem('panthera_hmi_page') || 'dashboard';

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

  if (activePage === 'points') {
    if (!motionCatalog) {
      loadMotionCatalog();
    }
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
  if (state && state !== 'UNKNOWN' && state !== 'STOPPED') {
    return 'badge badge-ok';
  }
  return 'badge';
}

function deriveMode(context, state) {
  if (state === 'STOPPED') {
    return 'OFFLINE';
  }
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
    return {enabled: true, reason: '请求软件停止；危险时优先使用硬件急停'};
  }

  if (latestSnapshot?.debug?.active &&
      !['manual_mode', 'clear_estop', 'request_reset', 'restart_cleaning_motor'].includes(command)) {
    return {enabled: false, reason: '点位调试中，生产命令已锁定'};
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
    if (command === 'request_reset') {
      return {enabled: true, reason: '错误状态允许人工复位'};
    }
    if (command === 'restart_cleaning_motor') {
      return {enabled: true, reason: '清洁电机重新上电后，重连驱动并恢复空任务错误'};
    }
    return {enabled: false, reason: 'ERROR 中只允许复位或恢复清洁电机'};
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

  if (command === 'outlet_1_done' || command === 'outlet_2_done' ||
      command === 'actual_cycle_outlet_1' || command === 'actual_cycle_outlet_2') {
    const canStartPaused = state === 'PAUSED' && !hasActiveTask(context) &&
      ['IDLE', 'WAIT_DISCHARGE'].includes(context.paused_from_state);
    return {
      enabled: ['IDLE', 'WAIT_DISCHARGE'].includes(state) || canStartPaused,
      reason: '仅可在等待出料且没有进行中任务时触发',
    };
  }

  if (command === 'detection_done') {
    const canConfirmPaused = state === 'PAUSED' && hasActiveTask(context) &&
      context.paused_from_state === 'WAIT_DETECTION_DONE';
    return {
      enabled: state === 'WAIT_DETECTION_DONE' || canConfirmPaused,
      reason: state === 'WAIT_DETECTION_DONE' || canConfirmPaused
        ? '人工确认光谱检测完成'
        : '需要处于等待检测完成状态',
    };
  }

  return {enabled: true, reason: ''};
}

function renderServices(services, state, context) {
  const ready = coreServiceNames.filter((name) => services?.[name]?.ready).length;
  const missing = coreServiceNames.filter((name) => !services?.[name]?.ready);
  setText('serviceBadge', `核心服务 ${ready}/${coreServiceNames.length}`);
  const serviceBadge = $('serviceBadge');
  if (serviceBadge) {
    serviceBadge.className = ready === coreServiceNames.length ? 'badge badge-ok' : 'badge badge-warn';
    serviceBadge.title = missing.length ? `未就绪：${missing.join(', ')}` : '生产核心服务全部就绪';
  }

  document.querySelectorAll('button[data-command]').forEach((button) => {
    const command = button.dataset.command;
    const rule = commandRule(command, services, state, context);
    button.disabled = !rule.enabled || button.classList.contains('loading');
    button.title = rule.reason || button.title || '';
    button.setAttribute('aria-disabled', String(button.disabled));
  });
}

function renderWorkcellControl(control = {}) {
  const running = !!control.running;
  const busy = !!control.busy;
  const available = control.available !== false;
  const status = $('workcellStatus');
  if (status) {
    const operationLabel = {
      start: '启动',
      stop: '安全停止',
      restart: '安全重启',
    }[control.operation] || '操作';
    status.textContent = busy
      ? `${operationLabel}中…`
      : (running ? '工作站运行中' : '工作站已停止');
    status.className = `pill ${busy ? 'pill-warn' : (running ? 'pill-ok' : 'pill-warn')}`;
  }
  const logHint = control.last_exit_code && control.last_log
    ? `；日志：${control.last_log}`
    : '';
  setText('workcellMessage', `${control.message || 'HMI 常驻运行'}${logHint}`);
  const modeSelect = $('controlModeSelect');
  const activeMode = control.active_control_mode || '';
  const selectedMode = control.selected_control_mode || 'position_velocity';
  if (modeSelect) {
    if (!controlModeDirty) {
      modeSelect.value = selectedMode;
    }
    if (activeMode && modeSelect.value === activeMode) {
      controlModeDirty = false;
    }
    modeSelect.disabled = busy;
  }
  const modeLabels = {
    position_velocity: '速度位置模式',
    mit_gravity_compensation: 'MIT 重力补偿模式',
  };
  setText(
    'activeControlMode',
    activeMode ? `当前：${modeLabels[activeMode] || activeMode}` : '当前：工作站未启动');
  const start = $('startWorkcell');
  const stop = $('stopWorkcell');
  const restart = $('restartWorkcell');
  if (start) {
    start.disabled = !available || busy || running;
    start.title = !available ? '工作站脚本不可用' : (running ? '工作站已经运行' : '启动全部生产服务');
  }
  if (stop) {
    stop.disabled = !available || busy || !running;
    stop.title = running ? '等待安全状态、回 Home 后停止生产服务' : '工作站已经停止';
  }
  if (restart) {
    restart.disabled = !available || busy;
    restart.title = '先安全停止并回 Home，再重新启动全部生产服务';
  }
}

function setMotionCatalogResult(text, ok = true) {
  const box = $('motionCatalogResult');
  if (!box) {
    return;
  }
  box.textContent = text || '';
  box.classList.toggle('failed', !ok);
  box.classList.toggle('success', ok && !!text);
}

function motionPointTags(point) {
  return point && Array.isArray(point.tags) ? point.tags : [];
}

function isDailyTunableMotionPoint(point) {
  return motionPointTags(point).includes('tunable');
}

function setMotionPoseInputs(point) {
  const ids = [
    'motionPointX', 'motionPointY', 'motionPointZ',
    'motionPointRoll', 'motionPointPitch', 'motionPointYaw',
  ];
  const pose = point && point.pose;
  const validPose = pose && Array.isArray(pose.xyz) && pose.xyz.length === 3 &&
    Array.isArray(pose.rpy) && pose.rpy.length === 3;
  const values = validPose
    ? [
        ...pose.xyz.map((value) => Number(value) * 1000.0),
        ...pose.rpy.map((value) => Number(value) * 180.0 / Math.PI),
      ]
    : Array(6).fill('');
  ids.forEach((id, index) => {
    const input = $(id);
    if (!input) {
      return;
    }
    input.disabled = !validPose;
    input.value = validPose && Number.isFinite(values[index])
      ? String(Number(values[index].toFixed(index < 3 ? 3 : 2)))
      : '';
  });
  if (!validPose && point) {
    setMotionCatalogResult('该点是关节维护点；如需修改请展开高级 JSON。', true);
  }
}

function syncMotionPointForm() {
  if (!motionCatalog || !motionCatalogSelectedPoint) {
    return;
  }
  const point = motionCatalog.points[motionCatalogSelectedPoint];
  if (!point || !point.pose) {
    return;
  }
  const positionIds = ['motionPointX', 'motionPointY', 'motionPointZ'];
  const orientationIds = ['motionPointRoll', 'motionPointPitch', 'motionPointYaw'];
  const position = positionIds.map((id) => numericInputValue(id) / 1000.0);
  const orientation = orientationIds.map((id) => numericInputValue(id) * Math.PI / 180.0);
  point.pose.xyz = position;
  point.pose.rpy = orientation;
  const pointEditor = $('motionPointEditor');
  if (pointEditor) {
    pointEditor.value = JSON.stringify(point, null, 2);
  }
}

function syncMotionCatalogEditors() {
  if (!motionCatalog) {
    return;
  }
  const pointEditor = $('motionPointEditor');
  if (motionCatalogSelectedPoint && pointEditor) {
    motionCatalog.points[motionCatalogSelectedPoint] = JSON.parse(pointEditor.value);
  }
  syncMotionPointForm();
  const routeEditor = $('motionRouteEditor');
  if (motionCatalogSelectedRoute && routeEditor) {
    motionCatalog.routes[motionCatalogSelectedRoute] = JSON.parse(routeEditor.value);
  }
}

function renderMotionCatalog() {
  if (!motionCatalog) {
    return;
  }
  const pointSelect = $('motionPointSelect');
  const routeSelect = $('motionRouteSelect');
  const advancedToggle = $('motionShowAdvancedPoints');
  const allPointNames = Object.keys(motionCatalog.points || {}).sort();
  const dailyPointNames = allPointNames.filter(
    (name) => isDailyTunableMotionPoint(motionCatalog.points[name]));
  const pointNames = motionCatalogShowAdvancedPoints || dailyPointNames.length === 0
    ? allPointNames
    : dailyPointNames;
  const routeNames = Object.keys(motionCatalog.routes || {}).sort();

  if (advancedToggle) {
    advancedToggle.checked = motionCatalogShowAdvancedPoints;
  }

  if (pointSelect) {
    pointSelect.innerHTML = '';
    pointNames.forEach((name) => {
      const option = document.createElement('option');
      option.value = name;
      const point = motionCatalog.points[name] || {};
      option.textContent = isDailyTunableMotionPoint(point) ? `● ${name}` : name;
      option.title = point.description || name;
      pointSelect.appendChild(option);
    });
    if (!pointNames.includes(motionCatalogSelectedPoint)) {
      motionCatalogSelectedPoint = pointNames[0] || '';
    }
    pointSelect.value = motionCatalogSelectedPoint;
  }
  if (routeSelect) {
    routeSelect.innerHTML = '';
    routeNames.forEach((name) => {
      const option = document.createElement('option');
      option.value = name;
      option.textContent = name;
      routeSelect.appendChild(option);
    });
    if (!routeNames.includes(motionCatalogSelectedRoute)) {
      motionCatalogSelectedRoute = routeNames[0] || '';
    }
    routeSelect.value = motionCatalogSelectedRoute;
  }

  const pointEditor = $('motionPointEditor');
  const selectedPoint = motionCatalogSelectedPoint
    ? motionCatalog.points[motionCatalogSelectedPoint]
    : null;
  if (pointEditor) {
    pointEditor.value = motionCatalogSelectedPoint
      ? JSON.stringify(selectedPoint, null, 2)
      : '';
    pointEditor.disabled = !motionCatalogSelectedPoint;
  }
  setText('motionPointTitle', motionCatalogSelectedPoint || '未选择点位');
  setText('motionPointDescription', selectedPoint ? (selectedPoint.description || '无说明') : '--');
  setText(
    'motionPointTags',
    `分类: ${selectedPoint && motionPointTags(selectedPoint).length ? motionPointTags(selectedPoint).join(' / ') : '--'}`);
  setMotionPoseInputs(selectedPoint);
  const routeEditor = $('motionRouteEditor');
  if (routeEditor) {
    routeEditor.value = motionCatalogSelectedRoute
      ? JSON.stringify(motionCatalog.routes[motionCatalogSelectedRoute], null, 2)
      : '';
    routeEditor.disabled = !motionCatalogSelectedRoute;
  }
  const references = motionCatalogSelectedPoint
    ? (motionCatalogReferences[motionCatalogSelectedPoint] || [])
    : [];
  setText('motionPointReferences', `引用: ${references.length ? references.join(', ') : '无'}`);
}

async function loadMotionCatalog() {
  setMotionCatalogResult('读取固定轨迹目录...', true);
  try {
    const response = await fetch('/api/motion_catalog');
    const result = await response.json();
    if (!result.success) {
      throw new Error(result.message || '读取失败');
    }
    motionCatalog = result.catalog || {};
    motionCatalogReferences = result.references || {};
    motionCatalogDirty = false;
    setText('motionCatalogPath', result.path || '--');
    renderMotionCatalog();
    renderDebugPointOptions();
    setMotionCatalogResult(
      result.reload_available
        ? '轨迹目录已读取，Motion Server 可执行编译校验。'
        : '轨迹目录已读取，但 Motion Server 未启动，保存会被安全拒绝。',
      !!result.reload_available);
  } catch (error) {
    setMotionCatalogResult(`读取轨迹目录失败: ${error}`, false);
  }
}

function addMotionPoint() {
  if (!motionCatalog) {
    return;
  }
  const name = (window.prompt('新点位 ID（字母开头，只用字母、数字、下划线）') || '').trim();
  if (!name) {
    return;
  }
  if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(name)) {
    setMotionCatalogResult('点位 ID 格式不合法。', false);
    return;
  }
  if (motionCatalog.points[name]) {
    setMotionCatalogResult(`点位已存在: ${name}`, false);
    return;
  }
  motionCatalog.points[name] = {
    description: '新增工艺点；请先填写坐标，再保存并编译校验。',
    pose: {xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]},
    ik_seed: 'safe_joint_center',
    tags: ['process', 'tunable', 'commissioning'],
  };
  motionCatalogReferences[name] = [];
  motionCatalogSelectedPoint = name;
  motionCatalogDirty = true;
  renderMotionCatalog();
  setMotionCatalogResult('已新增点位草稿；请填写已记录的关节值或 pose 后再保存。', true);
}

function deleteMotionPoint() {
  if (!motionCatalogSelectedPoint || !motionCatalog) {
    return;
  }
  const references = motionCatalogReferences[motionCatalogSelectedPoint] || [];
  if (references.length) {
    setMotionCatalogResult(`不能删除，被以下位置引用: ${references.join(', ')}`, false);
    return;
  }
  if (!window.confirm(`确认删除未引用点位 ${motionCatalogSelectedPoint}？`)) {
    return;
  }
  delete motionCatalog.points[motionCatalogSelectedPoint];
  delete motionCatalogReferences[motionCatalogSelectedPoint];
  motionCatalogSelectedPoint = '';
  motionCatalogDirty = true;
  renderMotionCatalog();
}

function addMotionRoute() {
  if (!motionCatalog) {
    return;
  }
  const name = (window.prompt('新路线 ID（字母开头，只用字母、数字、下划线）') || '').trim();
  if (!name) {
    return;
  }
  if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(name) || motionCatalog.routes[name]) {
    setMotionCatalogResult('路线 ID 不合法或已经存在。', false);
    return;
  }
  const firstPoint = Object.keys(motionCatalog.points || {})[0];
  if (!firstPoint) {
    setMotionCatalogResult('至少需要一个点位才能新增路线。', false);
    return;
  }
  motionCatalog.routes[name] = {
    start: firstPoint,
    velocity_scale: 0.1,
    acceleration_scale: 0.1,
    enabled: false,
    segments: [{name: 'edit_me', type: 'joint', to: firstPoint}],
  };
  motionCatalogSelectedRoute = name;
  motionCatalogDirty = true;
  renderMotionCatalog();
  setMotionCatalogResult('已新增禁用路线草稿；编辑起点和轨迹段后再启用并保存。', true);
}

function deleteMotionRoute() {
  if (!motionCatalogSelectedRoute || !motionCatalog) {
    return;
  }
  if (!window.confirm(`确认删除路线 ${motionCatalogSelectedRoute}？`)) {
    return;
  }
  delete motionCatalog.routes[motionCatalogSelectedRoute];
  motionCatalogSelectedRoute = '';
  motionCatalogDirty = true;
  renderMotionCatalog();
}

async function saveMotionCatalog(button) {
  try {
    syncMotionCatalogEditors();
  } catch (error) {
    setMotionCatalogResult(`JSON 格式错误: ${error}`, false);
    return;
  }
  const confirmed = await confirmAction(
    '保存会进行引用、IK、限位、碰撞、垂直约束和整条路线编译。任何失败都会自动恢复旧目录。确认继续？',
    '校验并保存固定轨迹');
  if (!confirmed) {
    return;
  }
  setButtonFeedback(button, 'loading');
  setMotionCatalogResult('正在保存并编译全部启用路线...', true);
  try {
    const response = await fetch('/api/motion_catalog', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({catalog: motionCatalog}),
    });
    const result = await response.json();
    if (!result.success) {
      throw new Error(result.message || '保存或编译失败');
    }
    motionCatalogDirty = false;
    setMotionCatalogResult(`${result.message}；备份: ${result.backup_path || '--'}`, true);
    setButtonFeedback(button, 'success');
    await loadMotionCatalog();
  } catch (error) {
    setMotionCatalogResult(`轨迹目录未生效: ${error}`, false);
    setButtonFeedback(button, 'failed');
  } finally {
    if (button) {
      button.classList.remove('loading');
    }
  }
}

function numericInputValue(id) {
  const el = $(id);
  const value = el ? Number(el.value) : NaN;
  if (!Number.isFinite(value)) {
    throw new Error(`${id} 不是有效数字`);
  }
  return value;
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
    speedScalePercent = Math.max(20, Math.min(100, Math.round(percent)));
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
  setText('laserStable', laser.stable
    ? `${formatMm(laser.stable_distance_mm)} / ${formatMm(laser.span_mm)}`
    : `等待稳定 / ${formatMm(laser.span_mm)}`);
  setText('laserReference', formatMm(laser.reference_mm));
  setText('laserDelta', formatMm(laser.delta_mm));
  setText('laserAxis', formatMm(axis));
  setText('laserStatus', `${laser.valid ? 'valid' : 'invalid'} | ${context.laser_scan_phase || 'idle'} | ${laser.status || 'no status'} | ${laser.source || '--'}`);
  setText('laserAge', laser.age_sec !== null && laser.age_sec !== undefined && laser.age_sec > 2.0 ? `${laserAgeText} 延迟` : laserAgeText);
  setText('placeLaser', formatMm(context.spectrometer_position_before_place_mm));
  setText('pickLaser', formatMm(context.spectrometer_position_before_pick_mm));
  if (laser.last_calibration) {
    setText('laserCalibrationResult', laser.last_calibration);
  }
  const calibrateButton = $('calibrateLaser');
  if (calibrateButton && !calibrateButton.classList.contains('loading')) {
    const safeState = ['IDLE', 'WAIT_DISCHARGE', 'PAUSED'].includes(context.state);
    const safeContext = !context.has_active_task
      && !context.cup_in_gripper
      && !context.spectrometer_occupied;
    const freshLaser = laser.age_sec !== null
      && laser.age_sec !== undefined
      && laser.age_sec <= 1.0;
    calibrateButton.disabled = !(
      laser.valid && laser.stable && freshLaser && safeState && safeContext);
    calibrateButton.title = calibrateButton.disabled
      ? '需要激光稳定，且工作站无活动任务、无杯子占位'
      : '记录标准品位置的稳定激光距离';
  }
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

function renderDebugPointOptions() {
  const select = $('debugPointSelect');
  if (!select || !motionCatalog || !motionCatalog.points) {
    return;
  }
  const previous = select.value;
  const points = Object.entries(motionCatalog.points)
    .filter(([, point]) => Array.isArray(point.tags) && point.tags.includes('tunable') && point.pose)
    .sort(([left], [right]) => left.localeCompare(right));
  select.innerHTML = '';
  points.forEach(([name, point]) => {
    const option = document.createElement('option');
    option.value = name;
    option.textContent = point.description ? `${name} · ${point.description}` : name;
    select.appendChild(option);
  });
  if (points.some(([name]) => name === previous)) {
    select.value = previous;
  }
  if (latestSnapshot) {
    renderDebug(latestSnapshot);
  }
}

function setDebugResult(message, ok = true, warning = false) {
  const box = $('debugResult');
  if (!box) {
    return;
  }
  box.textContent = message || '--';
  box.classList.toggle('error-text', !ok);
  box.classList.toggle('warning-text', ok && warning);
}

function copyMeasuredPoseToTarget(snapshot = latestSnapshot) {
  const tool = (snapshot || {}).tool_pose || {};
  const position = tool.position_mm || {};
  const rpy = tool.rpy_deg || {};
  const values = {
    debugTargetX: position.x,
    debugTargetY: position.y,
    debugTargetZ: position.z,
    debugTargetRoll: rpy.roll,
    debugTargetPitch: rpy.pitch,
    debugTargetYaw: rpy.yaw,
  };
  if (!Object.values(values).every(Number.isFinite)) {
    setDebugResult('当前实测夹取中心无效，不能填入目标坐标。', false);
    return false;
  }
  Object.entries(values).forEach(([id, value]) => {
    $(id).value = Number(value).toFixed(2);
  });
  return true;
}

function renderDebug(snapshot) {
  const debug = snapshot.debug || {};
  const tool = snapshot.tool_pose || {};
  const active = !!debug.active;
  const sessionReady = active && debug.phase === 'ready' && !debugBusy;
  const pointReady = sessionReady && !!debug.selected_point;
  const status = $('debugStatus');
  if (status) {
    const phaseLabel = debugPhaseLabels[debug.phase] || debug.phase || '未进入调试';
    status.textContent = active && debug.selected_point
      ? `${phaseLabel} · ${debug.selected_point}`
      : phaseLabel;
    status.className = debug.phase === 'error'
      ? 'pill pill-error'
      : (sessionReady ? 'pill pill-ok' : 'pill pill-warn');
  }
  const select = $('debugPointSelect');
  if (select && debug.selected_point) {
    select.value = debug.selected_point;
  }
  if (select) {
    select.disabled = debugBusy || pointReady;
  }
  const enter = $('debugEnter');
  if (enter) {
    enter.disabled = active || debugBusy;
  }
  const goto = $('debugGoto');
  if (goto) {
    goto.disabled = !sessionReady || pointReady || !!debug.brush_enabled || !select || !select.value;
  }

  document.querySelectorAll('.jog-button').forEach((button) => {
    button.disabled = !pointReady;
  });
  document.querySelectorAll('.debug-target-grid input').forEach((input) => {
    input.disabled = !pointReady;
  });
  ['debugGripperOpen', 'debugGripperClose', 'debugBrushStart', 'debugBrushStop',
    'debugBrushSave', 'debugProcessTimingSave'].forEach((id) => {
    const button = $(id);
    if (button) {
      button.disabled = !sessionReady;
    }
  });
  const timingInputs = {
    debugBrushHoldSec: debug.brush_hold_sec,
    debugScanDurationSec: debug.scan_duration_sec,
  };
  Object.entries(timingInputs).forEach(([id, value]) => {
    const input = $(id);
    if (!input) {
      return;
    }
    input.disabled = !sessionReady;
    if (document.activeElement !== input && Number.isFinite(value)) {
      input.value = String(value);
    }
  });
  ['debugSave', 'debugCopyPose', 'debugMoveTo'].forEach((id) => {
    const button = $(id);
    if (button) {
      button.disabled = !pointReady;
    }
  });
  const exit = $('debugExit');
  if (exit) {
    exit.disabled = !active || debugBusy;
    exit.textContent = debug.selected_point
      ? '放弃未保存修改并回 Home'
      : '退出调试（不移动）';
  }
  document.querySelectorAll('.advanced-maintenance input, .advanced-maintenance select, .advanced-maintenance textarea, .advanced-maintenance button').forEach((control) => {
    control.disabled = active || debugBusy;
  });

  const position = tool.position_mm || {};
  const rpy = tool.rpy_deg || {};
  setText('debugPoseX', Number.isFinite(position.x) ? `${position.x.toFixed(2)} mm` : '--');
  setText('debugPoseY', Number.isFinite(position.y) ? `${position.y.toFixed(2)} mm` : '--');
  setText('debugPoseZ', Number.isFinite(position.z) ? `${position.z.toFixed(2)} mm` : '--');
  setText('debugPoseRoll', Number.isFinite(rpy.roll) ? `${rpy.roll.toFixed(2)}°` : '--');
  setText('debugPosePitch', Number.isFinite(rpy.pitch) ? `${rpy.pitch.toFixed(2)}°` : '--');
  setText('debugPoseYaw', Number.isFinite(rpy.yaw) ? `${rpy.yaw.toFixed(2)}°` : '--');
  setText('debugInterlock', pointReady
    ? `${debugPhaseLabels[debug.phase] || debug.phase} · 已点动 ${debug.jog_count || 0} 次 · ${debug.dirty ? '有未保存修改' : '点位与配置一致'}`
    : (active
      ? (debug.brush_enabled
        ? '毛刷运行中；夹爪仍可操作。停止毛刷后才能前往所选点位。'
        : '调试已进入，夹爪和毛刷可直接操作；需要示教时再前往所选点位。')
      : '点击“进入调试（不移动）”，机械臂不会移动。'));
  if (pointReady && debugTargetPoint !== debug.selected_point &&
      copyMeasuredPoseToTarget(snapshot)) {
    debugTargetPoint = debug.selected_point;
  } else if (!active) {
    debugTargetPoint = '';
  }
  if (debug.last_message) {
    setDebugResult(
      debug.last_message,
      debug.phase !== 'error',
      String(debug.last_message).includes('⚠'));
  }
}

async function postJson(path, body = {}) {
  const response = await fetch(path, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body),
  });
  const result = await response.json();
  if (!response.ok) {
    throw new Error(result.message || `HTTP ${response.status}`);
  }
  return result;
}

async function runWorkcellOperation(action, button) {
  const labels = {start: '启动', stop: '安全停止', restart: '安全重启'};
  const confirmations = {
    start: '确认机械臂和清洁电机已经上电、现场安全。现在启动全部工作站服务？',
    stop: '系统会等待当前任务结束并将机械臂回到 Home，然后关闭生产服务；HMI 会继续运行。确认安全停止？',
    restart: '系统会先安全回 Home 并关闭生产服务，再按所选控制模式完整重新启动。确认安全重启？',
  };
  if (!(await confirmAction(confirmations[action], `${labels[action]}工作站`))) {
    return;
  }
  setButtonFeedback(button, 'loading');
  try {
    const controlMode = $('controlModeSelect')?.value || 'position_velocity';
    const result = await postJson(
      `/api/workcell/${action}`,
      action === 'stop' ? {} : {control_mode: controlMode});
    setText('workcellMessage', result.message || `${labels[action]}已受理`);
    appendLog(`WORKCELL ${action}: ${result.message || '--'}`, 'log-ok');
  } catch (error) {
    setText('workcellMessage', String(error));
    appendLog(`WORKCELL ${action}: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
  } finally {
    button?.classList.remove('loading');
    try {
      render(await fetchSnapshot());
    } catch (_) {
      // The always-on HMI refreshes operation state on the next poll.
    }
  }
}

async function runDebugRequest(path, body, button) {
  if (debugBusy) {
    return null;
  }
  debugBusy = true;
  setButtonFeedback(button, 'loading');
  renderDebug(latestSnapshot || {});
  try {
    const result = await postJson(path, body);
    setDebugResult(result.message, !!result.success, !!result.warning);
    appendLog(
      `DEBUG ${path}: ${result.message || '--'}`,
      result.warning ? 'log-warn' : (result.success ? 'log-ok' : 'log-error'));
    setButtonFeedback(
      button,
      result.warning ? 'warning' : (result.success ? 'success' : 'failed'));
    return result;
  } catch (error) {
    setDebugResult(String(error), false);
    appendLog(`DEBUG ${path}: ${error}`, 'log-error');
    setButtonFeedback(button, 'failed');
    return {success: false, message: String(error)};
  } finally {
    debugBusy = false;
    if (button) {
      button.classList.remove('loading');
    }
    try {
      render(await fetchSnapshot());
    } catch (_) {
      renderDebug(latestSnapshot || {});
    }
  }
}

async function sendDebugJog(button) {
  const linearStep = Number($('debugLinearStep').value);
  const angularStep = Number($('debugAngularStep').value);
  const translation = [0, 0, 0];
  const rotation = [0, 0, 0];
  const sign = Number(button.dataset.jogSign);
  const linearAxes = {x: 0, y: 1, z: 2};
  const rotationAxes = {roll: 0, pitch: 1, yaw: 2};
  if (button.dataset.jogAxis) {
    if (!Number.isFinite(linearStep) || linearStep < 2 || linearStep > 20) {
      setDebugResult('MIT 微调的平移步进必须在 2–20 mm，推荐 2–5 mm。', false);
      return;
    }
    translation[linearAxes[button.dataset.jogAxis]] = sign * linearStep / 1000;
  } else {
    if (!Number.isFinite(angularStep) || angularStep < 0.1 || angularStep > 10) {
      setDebugResult('旋转步进必须在 0.1–10°。', false);
      return;
    }
    rotation[rotationAxes[button.dataset.jogRotation]] = sign * angularStep * Math.PI / 180;
  }
  const result = await runDebugRequest(
    '/api/debug/jog', {translation_m: translation, rotation_rad: rotation}, button);
  if (result && result.success && !result.already_at_target) {
    copyMeasuredPoseToTarget(latestSnapshot);
  }
}

async function moveDebugToInput(button) {
  const ids = [
    'debugTargetX', 'debugTargetY', 'debugTargetZ',
    'debugTargetRoll', 'debugTargetPitch', 'debugTargetYaw',
  ];
  const rawValues = ids.map((id) => $(id).value.trim());
  const values = rawValues.map(Number);
  if (rawValues.some((value) => value === '') || !values.every(Number.isFinite)) {
    setDebugResult('请完整填写有效的 XYZ 和 Roll/Pitch/Yaw。', false);
    return;
  }
  await runDebugRequest('/api/debug/move_to', {
    target_xyz_m: values.slice(0, 3).map((value) => value / 1000),
    target_rpy_rad: values.slice(3).map((value) => value * Math.PI / 180),
  }, button);
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

  const stateLabel = flowLabels[state] || state;
  const modeLabel = modeLabels[mode] || mode;
  setText('stateText', `${stateLabel} · ${state}`);
  setText('stateAge', formatAge(cell.state_age_sec));
  setText('stateDuration', `状态数据更新 ${formatAge(cell.state_age_sec)}`);
  setText('previousState', `上一步 ${context.previous_state || '--'}`);
  setText('transitionReason', `原因 ${context.last_transition_reason || '--'}`);
  setText('modeText', modeLabel);
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
    stateBadge.textContent = stateLabel;
    stateBadge.className = stateClass(state);
  }
  const modeBadge = $('modeBadge');
  if (modeBadge) {
    modeBadge.textContent = `模式 ${modeLabel}`;
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
  renderWorkcellControl(snapshot.workcell_control || {});
  renderMotion(snapshot.motion || {}, snapshot.services || {});
  renderDebug(snapshot);
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
  button.classList.remove('loading', 'success', 'warning', 'failed');
  if (state) {
    button.classList.add(state);
  }
  if (state === 'success' || state === 'warning' || state === 'failed') {
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

async function sendSpeedScale(percent, button) {
  const resultBox = $('commandResult');
  const numericPercent = Number(percent);
  if (!Number.isFinite(numericPercent) || numericPercent < 20 || numericPercent > 100) {
    if (resultBox) {
      resultBox.textContent = '速度必须在 20%–100% 之间。';
    }
    setButtonFeedback(button, 'failed');
    return;
  }
  const scale = numericPercent / 100.0;
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

function wireButtons() {
  const calibrateLaser = $('calibrateLaser');
  calibrateLaser?.addEventListener('click', async () => {
    if (!(await confirmAction(
      '确认光谱仪当前位于扫描标准品固定位置 X=162.0 mm。系统将保存当前稳定激光距离作为新基准，不会移动机械臂。',
      '校准激光传感器'))) {
      return;
    }
    setButtonFeedback(calibrateLaser, 'loading');
    try {
      const result = await postJson('/api/laser/calibrate', {
        confirm_standard_position: true,
      });
      if (!result.success) {
        throw new Error(result.message || '校准被拒绝');
      }
      setText('laserCalibrationResult', result.message || '校准成功');
      appendLog(result.message || 'laser calibration succeeded', 'log-success');
    } catch (error) {
      setText('laserCalibrationResult', `校准失败：${error}`);
      appendLog(`laser calibration failed: ${error}`, 'log-error');
    } finally {
      setButtonFeedback(calibrateLaser, 'idle');
    }
  });
  const controlModeSelect = $('controlModeSelect');
  controlModeSelect?.addEventListener('change', () => {
    controlModeDirty = true;
    setText(
      'workcellMessage',
      '控制模式将在下一次启动或安全重启时生效；调试与生产共用该模式。');
  });
  const workcellButtons = {
    start: $('startWorkcell'),
    stop: $('stopWorkcell'),
    restart: $('restartWorkcell'),
  };
  Object.entries(workcellButtons).forEach(([action, button]) => {
    button?.addEventListener('click', () => {
      if (!button.disabled) {
        runWorkcellOperation(action, button);
      }
    });
  });

  document.querySelectorAll('[data-page-target]').forEach((button) => {
    button.addEventListener('click', () => setActivePage(button.dataset.pageTarget));
  });

  document.querySelectorAll('.step-presets button').forEach((button) => {
    button.addEventListener('click', () => {
      const group = button.closest('.step-presets');
      const input = group ? $(group.dataset.stepTarget) : null;
      if (input) {
        input.value = button.dataset.stepValue;
      }
    });
  });

  const debugEnter = $('debugEnter');
  if (debugEnter) {
    debugEnter.addEventListener('click', async () => {
      if (!(await confirmAction(
        '将暂停自动流程并进入调试。机械臂不会移动，进入后可直接操作夹爪和毛刷。',
        '进入调试'))) {
        return;
      }
      await runDebugRequest('/api/debug/enter', {}, debugEnter);
    });
  }
  const debugGoto = $('debugGoto');
  if (debugGoto) {
    debugGoto.addEventListener('click', async () => {
      const point = $('debugPointSelect').value;
      if (!point || !(await confirmAction(
        `机械臂将先确认/恢复 Home，再经过安全调试点前往 ${point}。确认工位周围安全。`,
        '前往所选点位'))) {
        return;
      }
      await runDebugRequest('/api/debug/goto', {point_name: point}, debugGoto);
    });
  }

  document.querySelectorAll('.jog-button').forEach((button) => {
    button.addEventListener('click', () => {
      if (!button.disabled) {
        sendDebugJog(button);
      }
    });
  });

  const debugCopyPose = $('debugCopyPose');
  debugCopyPose?.addEventListener('click', () => {
    if (copyMeasuredPoseToTarget()) {
      setDebugResult('已填入当前实测值，可以直接修改后执行。');
    }
  });
  const debugMoveTo = $('debugMoveTo');
  debugMoveTo?.addEventListener('click', () => moveDebugToInput(debugMoveTo));
  document.querySelectorAll('.debug-target-grid input').forEach((input) => {
    input.addEventListener('keydown', (event) => {
      if (event.key === 'Enter' && !debugMoveTo.disabled) {
        debugMoveTo.click();
      }
    });
  });

  const gripperOpen = $('debugGripperOpen');
  const gripperClose = $('debugGripperClose');
  gripperOpen?.addEventListener('click', () => runDebugRequest(
    '/api/debug/gripper', {command: 'open'}, gripperOpen));
  gripperClose?.addEventListener('click', () => runDebugRequest(
    '/api/debug/gripper', {command: 'close'}, gripperClose));

  const brushSlider = $('debugBrushSpeed');
  brushSlider?.addEventListener('input', () => {
    setText('debugBrushSpeedValue', `${brushSlider.value}%`);
  });
  const brushStart = $('debugBrushStart');
  const brushStop = $('debugBrushStop');
  const brushSave = $('debugBrushSave');
  brushStart?.addEventListener('click', () => runDebugRequest(
    '/api/debug/brush', {enabled: true, speed_percent: Number(brushSlider.value)}, brushStart));
  brushStop?.addEventListener('click', () => runDebugRequest(
    '/api/debug/brush', {enabled: false, speed_percent: 0}, brushStop));
  brushSave?.addEventListener('click', () => runDebugRequest(
    '/api/debug/brush', {
      enabled: !!((latestSnapshot || {}).debug || {}).brush_enabled,
      speed_percent: Number(brushSlider.value),
      persist_default: true,
    }, brushSave));

  const processTimingSave = $('debugProcessTimingSave');
  processTimingSave?.addEventListener('click', () => runDebugRequest(
    '/api/debug/process_timing', {
      brush_hold_sec: Number($('debugBrushHoldSec').value),
      scan_duration_sec: Number($('debugScanDurationSec').value),
    }, processTimingSave));

  const debugSave = $('debugSave');
  debugSave?.addEventListener('click', () => runDebugRequest('/api/debug/save', {}, debugSave));
  const debugExit = $('debugExit');
  debugExit?.addEventListener('click', async () => {
    const debug = ((latestSnapshot || {}).debug || {});
    const message = !debug.selected_point
      ? '将停止毛刷并退出调试，机械臂不会移动，自动流程仍保持暂停。确认继续？'
      : (debug.dirty
        ? '未保存点位不会写入配置；机械臂将沿标定安全退出路线回到安全调试点和 Home。确认继续？'
        : '机械臂将沿标定退出路线回到安全调试点和 Home。确认继续？');
    if (await confirmAction(message, '退出点位调试')) {
      await runDebugRequest('/api/debug/exit', {}, debugExit);
    }
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

  const motionPointSelect = $('motionPointSelect');
  if (motionPointSelect) {
    motionPointSelect.addEventListener('change', () => {
      try {
        syncMotionCatalogEditors();
        motionCatalogSelectedPoint = motionPointSelect.value;
        renderMotionCatalog();
      } catch (error) {
        setMotionCatalogResult(`点位 JSON 格式错误: ${error}`, false);
      }
    });
  }
  const motionShowAdvancedPoints = $('motionShowAdvancedPoints');
  if (motionShowAdvancedPoints) {
    motionShowAdvancedPoints.addEventListener('change', () => {
      try {
        syncMotionCatalogEditors();
        motionCatalogShowAdvancedPoints = motionShowAdvancedPoints.checked;
        renderMotionCatalog();
      } catch (error) {
        setMotionCatalogResult(`点位格式错误: ${error}`, false);
      }
    });
  }
  [
    'motionPointX', 'motionPointY', 'motionPointZ',
    'motionPointRoll', 'motionPointPitch', 'motionPointYaw',
  ].forEach((id) => {
    const input = $(id);
    if (input) {
      input.addEventListener('change', () => {
        try {
          syncMotionPointForm();
          motionCatalogDirty = true;
          setMotionCatalogResult('点位草稿已修改；保存后会编译全部关联路线。', true);
        } catch (error) {
          setMotionCatalogResult(`点位数值错误: ${error}`, false);
        }
      });
    }
  });
  const motionRouteSelect = $('motionRouteSelect');
  if (motionRouteSelect) {
    motionRouteSelect.addEventListener('change', () => {
      try {
        syncMotionCatalogEditors();
        motionCatalogSelectedRoute = motionRouteSelect.value;
        renderMotionCatalog();
      } catch (error) {
        setMotionCatalogResult(`路线 JSON 格式错误: ${error}`, false);
      }
    });
  }
  ['motionPointEditor', 'motionRouteEditor'].forEach((id) => {
    const editor = $(id);
    if (editor) {
      editor.addEventListener('input', () => {
        motionCatalogDirty = true;
        setMotionCatalogResult('目录有未保存修改。', true);
      });
    }
  });
  const motionCatalogReload = $('motionCatalogReload');
  if (motionCatalogReload) {
    motionCatalogReload.addEventListener('click', loadMotionCatalog);
  }
  const motionCatalogSave = $('motionCatalogSave');
  if (motionCatalogSave) {
    motionCatalogSave.addEventListener('click', () => saveMotionCatalog(motionCatalogSave));
  }
  const motionPointAdd = $('motionPointAdd');
  if (motionPointAdd) {
    motionPointAdd.addEventListener('click', addMotionPoint);
  }
  const motionPointDelete = $('motionPointDelete');
  if (motionPointDelete) {
    motionPointDelete.addEventListener('click', deleteMotionPoint);
  }
  const motionRouteAdd = $('motionRouteAdd');
  if (motionRouteAdd) {
    motionRouteAdd.addEventListener('click', addMotionRoute);
  }
  const motionRouteDelete = $('motionRouteDelete');
  if (motionRouteDelete) {
    motionRouteDelete.addEventListener('click', deleteMotionRoute);
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
