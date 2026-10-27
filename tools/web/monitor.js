'use strict';
const $ = (id) => document.getElementById(id);
const CIRCUMFERENCE = 2 * Math.PI * 139;
let previousAngle = null;
let unwrappedAngle = 0;
let lastState = null;
let busy = false;
let localError = '';
let streamError = false;
let demoPending = false;
let calibrationPending = false;
let demoError = '';
let renderedSession = null;
let controlPending = false;
let controlError = '';
let settingsError = '';
let settingsNotice = '';
let nnDirty = false;
const PARAM_FIELDS = ['kp', 'ki', 'voltage', 'poskp', 'posmax', 'posmove'];
const DEFAULT_PROFILES = {
  speed: { kp: .008, ki: .284, voltage: 1, poskp: 4, posmax: 60, posmove: 0 },
  position: { kp: .040, ki: .650, voltage: 1, poskp: 15, posmax: 90, posmove: 500 },
};

// Only replace changed text. Telemetry stays "latest wins", without a frame queue.
function text(id, value) {
  const element = $(id);
  const content = String(value);
  if (element.textContent !== content) element.textContent = content;
}

function makeDial() {
  const ns = 'http://www.w3.org/2000/svg';
  for (let degree = 0; degree < 360; degree += 5) {
    const major = degree % 30 === 0;
    const angle = (degree - 90) * Math.PI / 180;
    const inner = major ? 119 : 124;
    const line = document.createElementNS(ns, 'line');
    for (const [key, value] of Object.entries({
      x1: 180 + Math.cos(angle) * inner, y1: 180 + Math.sin(angle) * inner,
      x2: 180 + Math.cos(angle) * 131, y2: 180 + Math.sin(angle) * 131,
    })) line.setAttribute(key, value);
    if (major) line.setAttribute('class', 'major');
    $('ticks').appendChild(line);
    if (major) {
      const label = document.createElementNS(ns, 'text');
      label.setAttribute('x', 180 + Math.cos(angle) * 105);
      label.setAttribute('y', 180 + Math.sin(angle) * 105);
      label.textContent = degree;
      $('labels').appendChild(label);
    }
  }
  $('arc').style.strokeDasharray = `${CIRCUMFERENCE} ${CIRCUMFERENCE}`;
  $('arc').style.strokeDashoffset = CIRCUMFERENCE;
}

async function api(path, options) {
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `请求失败（HTTP ${response.status}）`);
  return data;
}

function connectionMessage(message, bad = false, ready = false) {
  const status = $('connectionStatus');
  status.classList.toggle('bad', bad);
  status.classList.toggle('ready', ready && !bad);
  const content = status.querySelector('span');
  if (content.textContent !== message) content.textContent = message;
}

async function refreshPorts() {
  const select = $('portSelect');
  const selected = select.value;
  $('refreshButton').disabled = true;
  localError = '';
  try {
    const { ports } = await api('/api/ports');
    select.replaceChildren();
    if (!ports.length) {
      select.add(new Option('未发现串口，请检查USB–TTL连接', ''));
    } else {
      for (const port of ports) {
        const label = port.description && port.description !== 'n/a'
          ? `${port.device} · ${port.description}` : port.device;
        select.add(new Option(label, port.device));
      }
      // Keep an explicit choice; otherwise prefer the actual external adapter,
      // never the unrelated J-Link CDC UART.
      if (ports.some((port) => port.device === selected)) select.value = selected;
      else {
        const adapter = ports.find((port) => /CH343/i.test(port.description || ''));
        if (adapter) select.value = adapter.device;
      }
    }
  } catch (error) {
    select.replaceChildren(new Option('扫描失败，请重试', ''));
    localError = error.message;
    connectionMessage(localError, true);
  } finally {
    $('refreshButton').disabled = false;
    syncControls();
  }
}

function syncControls() {
  const connected = Boolean(lastState?.connected);
  const running = Boolean(lastState?.demo?.running) || demoPending;
  const calibrating = calibrationPending
    || (lastState?.demo?.running && lastState.demo.operation === 'calibration');
  $('connectButton').disabled = busy || controlPending || running || calibrating || (!connected && !$('portSelect').value);
  $('portSelect').disabled = busy || connected;
  const fresh = lastState?.updated_at && Date.now() / 1000 - lastState.updated_at < .5;
  $('demoButton').disabled = !connected || controlPending || running || calibrating || !fresh || streamError
    || lastState?.sensor !== 'ok' || !lastState?.calibrated || Boolean(lastState?.fault);
  $('calibrateButton').disabled = !connected || controlPending || running || calibrating || !fresh || streamError
    || lastState?.sensor !== 'ok' || Boolean(lastState?.fault);
  $('stopButton').disabled = !connected;
  const available = connected && !busy && !controlPending && !running && !calibrating;
  // Let the firmware ACK decide whether a target is accepted. Display freshness
  // is not a second motor-control interlock.
  $('speedButton').disabled = !available;
  $('positionButton').disabled = !available;
  for (const id of ['applyParameters', 'applyNn', 'nnEnabled']) $(id).disabled = !available;
  text('demoButton', running && !calibrating ? '演示中…' : '一键演示');
  text('calibrateButton', calibrating ? '校准中…' : '电机校准');
}

function loadProfile() {
  const name = $('tuningProfile').value;
  const values = lastState?.control?.profiles?.[name] || DEFAULT_PROFILES[name];
  for (const key of PARAM_FIELDS) $(`${key}Input`).value = values[key];
}

async function sendControl(payload, settings = false) {
  if (controlPending) return false;
  controlPending = true;
  if (settings) { settingsError = ''; settingsNotice = '正在应用…'; }
  else controlError = '';
  syncControls();
  try {
    const result = await api('/api/control', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    if (settings) {
      settingsNotice = payload.action === 'nn' ? 'NN设置已应用，电机保持停止。' : '参数已应用，电机保持停止。';
      if (payload.action === 'nn') nnDirty = false;
    }
    renderState(result.state || await api('/api/state'));
    return true;
  } catch (error) {
    if (settings) { settingsError = error.message; settingsNotice = ''; }
    else controlError = error.message;
    return false;
  } finally {
    controlPending = false;
    syncControls();
  }
}

function applyParameters(event) {
  event.preventDefault();
  const values = Object.fromEntries(PARAM_FIELDS.map((key) => [key, Number($(`${key}Input`).value)]));
  if (values.posmove > 0 && values.posmove < 100) {
    settingsError = '轨迹时长为0或100至2000 ms';
    return;
  }
  const profile = $('tuningProfile').value;
  const message = `确认应用${profile === 'position' ? '位置环' : '速度环'}参数？\n`
    + `速度 Kp ${values.kp}，Ki ${values.ki}\n位置 Kp ${values.poskp}，限速 ${values.posmax} RPM\n`
    + `轨迹 ${values.posmove} ms，限压 ${values.voltage} V\n当前运行将停止，应用后不自动启动。`;
  if (!window.confirm(message)) return;
  sendControl({ action: 'parameters', profile, values, confirmed: true }, true);
}

function applyNn(event) {
  event.preventDefault();
  const enabled = $('nnEnabled').checked;
  const alpha = Number($('nnAlpha').value);
  if (!window.confirm(`确认${enabled ? '开启' : '关闭'}NN混合控制？\n`
    + `融合系数 ${alpha}。当前运行将停止，应用后不自动启动。`)) return;
  sendControl({ action: 'nn', enabled, alpha, confirmed: true }, true);
}

async function calibrateMotor() {
  if (calibrationPending || demoPending || lastState?.demo?.running) return;
  calibrationPending = true;
  demoError = '';
  syncControls();
  try {
    await api('/api/calibrate', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}',
    });
    renderState(await api('/api/state'));
  } catch (error) {
    demoError = error.message;
  } finally {
    calibrationPending = false;
    syncControls();
  }
}

async function demoAction(start) {
  if (start && (demoPending || calibrationPending || lastState?.demo?.running)) return;
  demoError = '';
  if (!start) controlError = '';
  if (start) demoPending = true;
  syncControls();
  try {
    await api(`/api/demo/${start ? 'start' : 'stop'}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}',
    });
    renderState(await api('/api/state'));
  } catch (error) {
    demoError = error.message;
  } finally {
    demoPending = false;
    syncControls();
  }
}

async function toggleConnection() {
  if (busy) return;
  busy = true;
  localError = '';
  demoError = '';
  syncControls();
  try {
    if (lastState?.connected) {
      await api('/api/disconnect', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
    } else {
      const port = $('portSelect').value;
      if (!port) throw new Error('请选择外置USB–TTL串口。');
      // Opening a serial port never starts or calibrates the motor.
      await api('/api/connect', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ port }) });
      previousAngle = null;
      unwrappedAngle = 0;
    }
    renderState(await api('/api/state'));
  } catch (error) {
    localError = error.message;
    connectionMessage(localError, true);
  } finally {
    busy = false;
    syncControls();
  }
}

function renderState(state) {
  const session = JSON.stringify([state.connected, state.port, state.connection_id]);
  if (renderedSession !== session) {
    demoError = '';
    controlError = '';
    settingsError = '';
    settingsNotice = '';
    nnDirty = false;
    lastState = state;
    loadProfile();
  }
  renderedSession = session;
  lastState = state;
  const connected = Boolean(state.connected);
  const hasData = state.updated_at !== null && state.updated_at !== undefined;
  const fresh = hasData ? Math.max(0, Date.now() / 1000 - state.updated_at) : Infinity;
  const sensorOk = connected && state.sensor === 'ok' && fresh < 1 && !state.error && !streamError;
  text('liveText', sensorOk ? '实时接收中' : !connected ? '设备未连接'
    : streamError ? '网页链路重连中' : hasData && fresh >= 1 ? '数据已过期' : '等待有效遥测');
  $('liveBadge').classList.toggle('online', sensorOk);
  $('liveBadge').classList.toggle('stale', connected && !sensorOk);
  text('connectText', busy ? '处理中…' : connected ? '断开连接' : '连接设备');
  $('connectButton').classList.toggle('secondary', connected);
  syncControls();
  const demo = state.demo || {};
  text('demoStatus', !connected ? '待命 · 请先连接设备' : demoError || demo.error
    || (demo.running ? demo.phase : !sensorOk ? '待命 · 等待有效编码器数据'
      : state.motor_mode === 1 ? '校准中'
      : !state.calibrated ? '请先点击电机校准' : demo.phase || '待命'));
  $('demoStatus').classList.toggle('bad', connected && Boolean(demoError || demo.error));
  const control = state.control || {};
  const manual = control.manual || {};
  text('motorMode', !connected ? '待连接' : ({ 0: '已停止', 1: '校准中', 2: '速度环', 3: '故障', 8: '位置环' }[state.motor_mode] || '待命'));
  text('controlStatus', !connected ? '连接后可下发控制指令' : controlError
    || (controlPending ? '指令下发中…' : manual.message || '待命'));
  $('controlStatus').classList.toggle('bad', Boolean(controlError));
  text('settingsStatus', settingsError || settingsNotice || '仅应用时停机，不会自动重新启动。');
  $('settingsStatus').classList.toggle('bad', Boolean(settingsError));
  const nnKnown = control.nn_mode !== null && control.nn_mode !== undefined;
  const nnOn = control.nn_mode === 2;
  text('nnState', (nnKnown ? nnOn ? `已开启 · ${Number(control.nn_alpha || 0).toFixed(3)}` : '已关闭'
    : '未下发') + (nnDirty ? ' · 待应用' : ''));
  $('nnState').classList.toggle('on', nnOn);
  if (!nnDirty) {
    $('nnEnabled').checked = nnOn;
    if (control.nn_alpha !== null && control.nn_alpha !== undefined) $('nnAlpha').value = control.nn_alpha;
  }
  text('rpmFeedback', sensorOk ? (Number(state.speed_mrad_s || 0) * 60 / (2000 * Math.PI)).toFixed(1) : '—');
  text('uqFeedback', sensorOk && state.uq_mv !== null && state.uq_mv !== undefined ? (state.uq_mv / 1000).toFixed(3) : '—');

  if (localError) connectionMessage(localError, true);
  else if (state.error) connectionMessage(`串口读取异常：${state.error}`, true);
  else if (streamError) connectionMessage('数据推送中断，正在自动重连…', true);
  else connectionMessage(connected ? `${state.port} · ${state.baud} baud` : '未连接设备', false, sensorOk);

  text('sensorChip', sensorOk ? 'AS5600 · 正常' : !connected ? 'AS5600 · 待连接'
    : streamError ? '推送重连中' : !hasData ? '等待传感器'
    : fresh >= 1 ? '数据已过期' : '传感器异常');
  $('sensorChip').classList.toggle('ok', sensorOk);
  $('sensorChip').classList.toggle('error', connected && (state.sensor === 'error' || Boolean(state.error)));
  text('linkState', sensorOk ? '数据正常' : !connected ? '等待设备' : '等待更新');
  $('linkState').classList.toggle('ok', sensorOk);
  text('rateValue', sensorOk ? Number(state.rx_hz || 0).toFixed(1) : '—');
  text('droppedValue', hasData ? state.dropped_frames ?? 0 : '—');
  text('detailStatus', state.error ? `串口读取异常：${state.error}`
    : !connected ? '连接后显示通信状态。'
    : !hasData ? `串口已打开，尚未收到有效角度帧（已接收 ${state.rx_bytes || 0} 字节）。`
    : sensorOk ? `帧间隔 P95 ${Number(state.interval_p95_ms || 0).toFixed(1)} ms，最大 ${Number(state.interval_max_ms || 0).toFixed(1)} ms。`
    : streamError ? '网页推送中断；显示值为最后一次接收，不代表当前电机状态。'
    : fresh >= 1 ? '遥测已停止更新；当前显示最后一次接收值，请检查固件运行状态。'
    : '固件报告传感器读取异常，数值不可作为有效反馈。');

  if (!hasData) {
    for (const id of ['angleValue', 'rawValue', 'speedValue', 'radianValue', 'ageValue']) text(id, '—');
    text('direction', connected ? '等待角度数据…' : '连接设备后显示实时转动');
    previousAngle = null;
    unwrappedAngle = 0;
    $('pointer').style.transform = 'rotate(0deg)';
    $('rotor').style.transform = 'rotate(0deg)';
    $('arc').style.strokeDashoffset = CIRCUMFERENCE;
    return;
  }
  if (state.angle_cdeg === null || state.angle_cdeg === undefined) return;
  const angle = state.angle_cdeg / 100;
  const speed = state.speed_mrad_s / 1000;
  text('angleValue', angle.toFixed(2));
  text('rawValue', state.raw);
  text('speedValue', speed.toFixed(2));
  text('radianValue', (state.angle_mrad / 1000).toFixed(3));
  text('ageValue', state.interval_mean_ms == null ? '—' : state.interval_mean_ms.toFixed(1));
  const delta = previousAngle === null ? 0 : ((angle - previousAngle + 540) % 360) - 180;
  unwrappedAngle += previousAngle === null ? angle : delta;
  previousAngle = angle;
  $('pointer').style.transform = `rotate(${unwrappedAngle}deg)`;
  $('rotor').style.transform = `rotate(${unwrappedAngle}deg)`;
  $('arc').style.strokeDashoffset = CIRCUMFERENCE * (1 - angle / 360);
  // Encoder-positive direction is not necessarily clockwise from the camera.
  text('direction', !connected ? '已断开 · 显示最后一次接收值' : !sensorOk ? '数据未更新 · 保留最后读数'
    : Math.abs(speed) < .04 ? '转子静止'
    : `${speed > 0 ? '编码器正向' : '编码器反向'} · ${Math.abs(speed).toFixed(2)} rad/s`);
}

makeDial();
refreshPorts();
$('refreshButton').addEventListener('click', refreshPorts);
$('portSelect').addEventListener('change', syncControls);
$('connectButton').addEventListener('click', toggleConnection);
$('demoButton').addEventListener('click', () => demoAction(true));
$('calibrateButton').addEventListener('click', calibrateMotor);
$('stopButton').addEventListener('click', () => demoAction(false));
$('speedForm').addEventListener('submit', (event) => {
  event.preventDefault();
  sendControl({ action: 'run', rpm: Number($('rpmInput').value) });
});
$('positionForm').addEventListener('submit', (event) => {
  event.preventDefault();
  sendControl({ action: 'position', degrees: Number($('positionInput').value),
    reference: $('positionReference').value });
});
$('tuningProfile').addEventListener('change', loadProfile);
$('tuningForm').addEventListener('submit', applyParameters);
$('nnForm').addEventListener('submit', applyNn);
$('nnEnabled').addEventListener('change', () => { nnDirty = true; });
$('nnAlpha').addEventListener('input', () => { nnDirty = true; });
const stream = new EventSource('/api/events');
stream.onmessage = (event) => {
  try {
    lastState = JSON.parse(event.data);
    streamError = false;
  } catch {
    streamError = true;
    connectionMessage('遥测数据格式异常，等待下一帧…', true);
  }
};
stream.onerror = () => {
  streamError = true;
  connectionMessage('数据推送中断，正在自动重连…', true);
};
async function initialState() {
  try { lastState = await api('/api/state'); }
  catch (error) { localError = error.message; connectionMessage(localError, true); }
}
initialState();
function frame() {
  if (lastState) renderState(lastState);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
