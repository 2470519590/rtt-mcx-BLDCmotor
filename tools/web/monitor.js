'use strict';
const $ = (id) => document.getElementById(id);
const CIRCUMFERENCE = 2 * Math.PI * 139;
let previousAngle = null;
let unwrappedAngle = 0;
let lastState = null;
let busy = false;
let localError = '';
let streamError = false;

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
  $('connectButton').disabled = busy || (!connected && !$('portSelect').value);
  $('portSelect').disabled = busy || connected;
}

async function toggleConnection() {
  if (busy) return;
  busy = true;
  localError = '';
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
    : !connected ? '连接串口后显示链路统计。校准与调参请使用CLI。'
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
