/* Headless UI regression. Mock data only; never opens a serial port.
   Requires Playwright via normal node resolution or NODE_PATH.
   Run: node tools/test_web_ui.cjs [http://127.0.0.1:<port>]
   Without a URL an isolated static HTTP server serves tools/web locally. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const http = require('node:http');
const { chromium } = require('playwright');

const root = path.resolve(__dirname, '..');
const web = path.join(__dirname, 'web');
const output = path.join(root, 'debug', 'web-ui');
fs.mkdirSync(output, { recursive: true });

const blank = {
  connected: false, port: null, baud: 115200, error: null,
  raw: null, angle_mrad: null, angle_cdeg: null, speed_mrad_s: null,
  sensor: 'waiting', updated_at: null, revision: 0, rx_bytes: 0,
};
const online = (angle = 135) => ({
  ...blank, connected: true, port: 'COM32', raw: Math.round(angle / 360 * 4096),
  angle_mrad: Math.round(angle / 180 * Math.PI * 1000), angle_cdeg: angle * 100,
  speed_mrad_s: 1571, sensor: 'ok', updated_at: Date.now() / 1000,
  revision: 5, rx_hz: 99.8, interval_mean_ms: 10, interval_p95_ms: 10.8,
  interval_max_ms: 12.1, dropped_frames: 0, calibrated: 1, fault: 0,
  motor_mode: 0, uq_mv: 0,
  control: {
    profiles: {
      speed: { kp: .008, ki: .284, voltage: 1, poskp: 4, posmax: 60, posmove: 0 },
      position: { kp: .040, ki: .650, voltage: 1, poskp: 15, posmax: 90, posmove: 500 },
    },
    applied: {}, nn_mode: null, nn_alpha: null,
    manual: { active: false, kind: null, target: null, message: '待命' },
  },
  demo: { running: false, phase: '待命', error: null },
});

async function main() {
  let server;
  let browser;
  let count = 0;
  const check = (value, message) => { assert.ok(value, message); count++; };
  try {
    let base = process.argv[2];
    if (!base) {
      server = http.createServer((req, res) => {
        const filename = ({ '/': 'index.html', '/styles.css': 'styles.css',
          '/monitor.js': 'monitor.js' })[req.url];
        if (!filename) { res.writeHead(404); return res.end(); }
        const type = filename.endsWith('.css') ? 'text/css' :
          filename.endsWith('.js') ? 'text/javascript' : 'text/html';
        res.writeHead(200, { 'Content-Type': type + '; charset=utf-8' });
        res.end(fs.readFileSync(path.join(web, filename)));
      });
      await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
      base = `http://127.0.0.1:${server.address().port}`;
    }
    const launchOptions = { headless: true };
    if (process.env.PLAYWRIGHT_CHANNEL) launchOptions.channel = process.env.PLAYWRIGHT_CHANNEL;
    else if (!fs.existsSync(chromium.executablePath())) launchOptions.channel = 'msedge';
    else launchOptions.executablePath = chromium.executablePath();
    browser = await chromium.launch(launchOptions);
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 },
      reducedMotion: 'reduce' });
    const errors = [];
    page.on('pageerror', (e) => errors.push(e.message));
    let state = { ...blank };
    let ports = [{ device: 'COM21', description: 'J-Link CDC UART' },
      { device: 'COM32', description: 'USB-Enhanced-SERIAL CH343' }];
    let rejectConnect = false;
    let rejectDemo = false;
    let rejectControl = false;
    const requests = [];
    await page.route('**/api/**', async (route) => {
      const req = route.request();
      const endpoint = new URL(req.url()).pathname;
      requests.push({ endpoint, method: req.method(), data: req.postData() });
      let data;
      let status = 200;
      if (endpoint === '/api/ports') data = { ports };
      else if (endpoint === '/api/state') data = state;
      else if (endpoint === '/api/connect') {
        if (rejectConnect) { data = { error: '串口被其他程序占用' }; status = 400; }
        else { state = online(); data = { ok: true }; }
      } else if (endpoint === '/api/disconnect') { state = { ...blank }; data = { ok: true }; }
      else if (endpoint === '/api/demo/start') {
        if (rejectDemo) { data = { error: '本次演示请求失败' }; status = 400; }
        else {
          state = { ...online(), demo: { running: true, phase: '速度环 · 60 RPM' } };
          data = { ok: true };
        }
      } else if (endpoint === '/api/demo/stop') {
        state = { ...online(), demo: { running: false, phase: '演示已停止' } };
        data = { ok: true };
      } else if (endpoint === '/api/calibrate') {
        state = { ...online(), calibrated: 0, motor_mode: 1,
          demo: { running: true, phase: '校准中', operation: 'calibration' } };
        data = { ok: true };
      } else if (endpoint === '/api/control') {
        if (rejectControl) { status = 400; data = { error: '固件拒绝测试命令' }; }
        else {
          const command = JSON.parse(req.postData());
          const control = structuredClone(state.control || online().control);
          if (command.action === 'parameters') {
            control.profiles[command.profile] = command.values;
            control.applied = command.values;
            control.manual = { active: false, message: '参数已应用，电机保持停止' };
          } else if (command.action === 'nn') {
            control.nn_mode = command.enabled ? 2 : 0;
            control.nn_alpha = command.alpha;
            control.manual = { active: false, message: 'NN设置已应用，电机保持停止' };
          } else if (command.action === 'run' || command.action === 'position') {
            control.manual = { active: true, kind: command.action,
              message: command.action === 'run' ? `速度目标 ${command.rpm} RPM` : `位置目标 ${command.degrees}°` };
          }
          state = { ...online(), control };
          data = { ok: true, state };
        }
      }
      else { status = 500; data = { error: 'Unexpected endpoint' }; }
      await route.fulfill({ status, contentType: 'application/json', body: JSON.stringify(data) });
    });
    await page.addInitScript(() => {
      // Replaces only the browser test stream, not the shipped application.
      window.EventSource = class {
        constructor() { window.testStream = this; }
      };
    });
    await page.goto(base);
    await page.waitForFunction(() => document.querySelector('#portSelect').value === 'COM32');
    const emit = async (data) => {
      await page.evaluate((sample) => window.testStream.onmessage({ data: JSON.stringify(sample) }), data);
      await page.waitForTimeout(70);
    };
    await page.waitForTimeout(80);
    check(await page.locator('#angleValue').textContent() === '—', 'no fabricated angle at startup');
    check(await page.locator('#liveText').textContent() === '设备未连接', 'disconnected state');
    check(await page.locator('.ticks line').count() === 72, 'dial has 72 ticks');
    check(await page.locator('.labels text').count() === 12, 'dial has 12 labels');
    check(await page.locator('h1').count() === 1, 'one document title');
    check(await page.locator('#demoButton').isDisabled(), 'demo unavailable when disconnected');
    check(await page.locator('#calibrateButton').isDisabled(), 'calibration unavailable without serial');
    check(await page.locator('footer, .workspace-note').count() === 0, 'no filler footer or disclaimer');
    check(!(await page.locator('body').textContent()).includes('CLI'), 'no CLI detour in presentation UI');
    check(await page.locator('#stopButton').isDisabled(), 'stop unavailable without serial');
    check(await page.locator('#speedButton').isDisabled(), 'manual speed unavailable without serial');
    check(await page.locator('#positionButton').isDisabled(), 'manual position unavailable without serial');
    check(await page.locator('#applyParameters').isDisabled(), 'tuning unavailable without serial');
    check(await page.locator('#applyNn').isDisabled(), 'NN unavailable without serial');
    check(await page.locator('label[for=portSelect]').count() === 1, 'port selector labeled');
    await page.screenshot({ path: path.join(output, 'desktop-disconnected.png'), fullPage: true });
    await page.locator('#connectButton').click();
    await page.waitForFunction(() => document.querySelector('#liveText').textContent === '实时接收中');
    check(await page.locator('#angleValue').textContent() === '135.00', 'degrees rendered');
    check(await page.locator('#speedValue').textContent() === '1.57', 'velocity rendered');
    check(await page.locator('#radianValue').textContent() === '2.356', 'radians rendered');
    check(await page.locator('#rawValue').textContent() === '1536', 'raw angle rendered');
    check(await page.locator('#ageValue').textContent() === '10.0', 'interval rendered');
    check(await page.locator('#rateValue').textContent() === '99.8', 'rate rendered');
    check(await page.locator('#portSelect').isDisabled(), 'port selection locked when connected');
    check(await page.locator('#demoButton').isEnabled(), 'calibrated live device can start demo');
    check(await page.locator('#speedButton').isEnabled(), 'manual speed available');
    check(await page.locator('#positionButton').isEnabled(), 'manual position available');
    check(await page.locator('#nnState').textContent() === '未下发', 'unknown NN is not fabricated as off');
    const controlRequests = (action) => requests.filter((r) => r.endpoint === '/api/control'
      && JSON.parse(r.data).action === action);
    await page.locator('#rpmInput').fill('-60');
    await page.locator('#speedButton').click();
    await page.waitForFunction(() => document.querySelector('#controlStatus').textContent.includes('-60'));
    check(JSON.parse(controlRequests('run').at(-1).data).rpm === -60, 'reverse speed sent in RPM');
    await page.waitForTimeout(300);
    check(controlRequests('heartbeat').length === 0, 'manual keepalive belongs to server, not browser timers');
    await page.locator('#stopButton').click();
    await emit(online());
    await page.locator('#positionInput').fill('120');
    await page.locator('#positionButton').click();
    await page.waitForFunction(() => document.querySelector('#controlStatus').textContent.includes('120'));
    check(JSON.parse(controlRequests('position').at(-1).data).reference === 'relative',
      'position defaults to explicit relative reference');
    await page.locator('#stopButton').click();
    await emit(online());
    await page.locator('#positionReference').selectOption('absolute');
    await page.locator('#positionInput').fill('-30');
    await page.locator('#positionButton').click();
    await page.waitForFunction(() => document.querySelector('#controlStatus').textContent.includes('-30'));
    check(JSON.parse(controlRequests('position').at(-1).data).reference === 'absolute',
      'absolute calibration-zero position supported');
    await page.locator('#stopButton').click();
    await emit(online());
    check(await page.locator('#posmoveInput').inputValue() === '500', 'tested smooth position profile shown');
    await page.locator('#tuningProfile').selectOption('speed');
    check(await page.locator('#kpInput').inputValue() === '0.008', 'speed profile has separate gains');
    await page.locator('#tuningProfile').selectOption('position');
    await page.locator('#kpInput').fill('0.051');
    const paramsBefore = controlRequests('parameters').length;
    page.once('dialog', (dialog) => dialog.dismiss());
    await page.locator('#applyParameters').click();
    check(controlRequests('parameters').length === paramsBefore, 'cancel tuning sends no parameter request');
    let parameterConfirmation = '';
    await emit({ ...state, updated_at: Date.now() / 1000 });
    page.once('dialog', async (dialog) => { parameterConfirmation = dialog.message(); await dialog.accept(); });
    await page.locator('#applyParameters').click();
    await page.waitForFunction(() => document.querySelector('#settingsStatus').textContent.includes('参数已应用'));
    const parameterRequest = JSON.parse(controlRequests('parameters').at(-1).data);
    check(parameterRequest.confirmed === true && parameterRequest.values.kp === .051,
      'confirmed tuning uses visible units and values');
    check(parameterConfirmation.includes('0.051') && parameterConfirmation.includes('停止'),
      'confirmation lists actual settings and stop behavior');
    check(await page.locator('#kpInput').inputValue() === '0.051', 'telemetry does not overwrite edited values');
    await emit({ ...state, updated_at: Date.now() / 1000 });
    await page.locator('#nnEnabled').check();
    const nnBefore = controlRequests('nn').length;
    page.once('dialog', (dialog) => dialog.dismiss());
    await page.locator('#applyNn').click();
    check(controlRequests('nn').length === nnBefore, 'cancel NN sends no mode change');
    await emit({ ...state, updated_at: Date.now() / 1000 });
    page.once('dialog', (dialog) => dialog.accept());
    await page.locator('#applyNn').click();
    await page.waitForFunction(() => document.querySelector('#nnState').textContent.includes('已开启'));
    const nnRequest = JSON.parse(controlRequests('nn').at(-1).data);
    check(nnRequest.confirmed === true && nnRequest.enabled && nnRequest.alpha === .25,
      'NN enable includes nonzero blend and confirmation');
    check(await page.locator('#nnEnabled').isChecked(), 'NN switch follows acknowledged state');
    await emit({ ...state, updated_at: Date.now() / 1000 });
    await page.locator('#nnEnabled').uncheck();
    page.once('dialog', (dialog) => dialog.accept());
    await page.locator('#applyNn').click();
    await page.waitForFunction(() => document.querySelector('#nnState').textContent === '已关闭');
    check(JSON.parse(controlRequests('nn').at(-1).data).enabled === false, 'NN off is explicitly sent');
    rejectControl = true;
    await emit({ ...state, updated_at: Date.now() / 1000 });
    await page.locator('#speedButton').click();
    await page.waitForFunction(() => document.querySelector('#controlStatus').textContent.includes('固件拒绝'));
    check(await page.locator('#controlStatus').evaluate((el) => el.classList.contains('bad')),
      'rejected motor command is visibly unsuccessful');
    rejectControl = false;
    await emit(online());
    await page.locator('#demoButton').click();
    await page.waitForFunction(() => document.querySelector('#demoStatus').textContent.includes('速度环'));
    check(await page.locator('#demoButton').isDisabled(), 'no duplicate demo start');
    check(await page.locator('#stopButton').isEnabled(), 'stop available during demo');
    check(await page.locator('#connectButton').isDisabled(), 'no disconnect race during demo');
    check(await page.locator('#speedButton').isDisabled(), 'manual speed cannot race demo');
    check(await page.locator('#applyParameters').isDisabled(), 'parameters cannot race demo');
    await page.locator('#stopButton').click();
    await page.waitForFunction(() => document.querySelector('#demoStatus').textContent === '演示已停止');
    check(await page.locator('#demoButton').isEnabled(), 'cancelled demo can run again');
    await emit({ ...online(), calibrated: 0 });
    check(await page.locator('#demoButton').isDisabled(), 'uncalibrated device cannot start');
    check(await page.locator('#positionButton').isEnabled(), 'position delegates acceptance to firmware ACK');
    check(await page.locator('#speedButton').isEnabled(), 'speed delegates acceptance to firmware ACK');
    await emit({ ...online(), updated_at: 1, sensor: 'error', calibrated: 0, fault: 2 });
    check(await page.locator('#positionButton').isEnabled(), 'stale display does not block position submission');
    check(await page.locator('#speedButton').isEnabled(), 'stale display does not block speed submission');
    await emit({ ...online(), calibrated: 0 });
    check(await page.locator('#calibrateButton').isEnabled(), 'calibration is accessible for uncalibrated device');
    check(await page.locator('#demoStatus').textContent() === '请先点击电机校准',
      'calibration instruction points to visible button');
    await page.locator('#calibrateButton').click();
    await page.waitForFunction(() => document.querySelector('#calibrateButton').textContent === '校准中…');
    check(await page.locator('#calibrateButton').isDisabled(), 'no duplicate calibration');
    check(await page.locator('#demoButton').isDisabled(), 'demo waits for calibration completion');
    check(await page.locator('#stopButton').isEnabled(), 'calibration can be stopped');
    check(await page.locator('#connectButton').isDisabled(), 'connection remains during calibration');
    await emit({ ...online(), motor_mode: 0,
      demo: { running: false, phase: '校准完成', operation: 'calibration' } });
    check(await page.locator('#demoButton').isEnabled(), 'demo becomes available from firmware calibrated state');
    check(await page.locator('#demoStatus').textContent() === '校准完成', 'calibration completion visible');
    await emit(online());
    rejectDemo = true;
    await page.locator('#demoButton').click();
    await page.waitForFunction(() => document.querySelector('#demoStatus').textContent === '本次演示请求失败');
    check(await page.locator('#demoStatus').evaluate((el) => el.classList.contains('bad')),
      'current demo request error remains visible');
    await page.locator('#connectButton').click();
    await page.waitForFunction(() => document.querySelector('#liveText').textContent === '设备未连接');
    check(await page.locator('#demoStatus').textContent() === '待命 · 请先连接设备',
      'disconnect clears local demo error');
    await page.locator('#connectButton').click();
    await page.waitForFunction(() => document.querySelector('#liveText').textContent === '实时接收中');
    check(await page.locator('#demoStatus').textContent() === '待命',
      'reconnect does not carry previous demo error');
    await page.locator('#demoButton').click();
    await page.waitForFunction(() => document.querySelector('#demoStatus').textContent === '本次演示请求失败');
    await emit({ ...online(), connection_id: 2 });
    check(await page.locator('#demoStatus').textContent() === '待命',
      'new server serial session clears local error without a button click');
    rejectDemo = false;
    await emit({ ...online(), sensor: 'error' });
    check(await page.locator('#demoStatus').textContent() === '待命 · 等待有效编码器数据',
      'sensor error is not misreported as firmware or calibration problem');
    await emit(online());
    check((await page.locator('#direction').textContent()).includes('编码器正向'), 'no false physical clockwise claim');
    await emit(online(359));
    const before = await page.locator('#pointer').evaluate((el) => el.style.transform);
    await emit(online(1));
    const after = await page.locator('#pointer').evaluate((el) => el.style.transform);
    const angle = (transform) => Number(transform.match(/rotate\(([-.\d]+)deg\)/)[1]);
    check(Math.abs(angle(after) - angle(before) - 2) < .01, '359 -> 1 unwraps by +2 degrees');
    await emit({ ...online(135), speed_mrad_s: -1571 });
    check((await page.locator('#direction').textContent()).includes('编码器反向'), 'reverse indication');
    await emit({ ...online(), speed_mrad_s: 0 });
    check(await page.locator('#direction').textContent() === '转子静止', 'stationary indication');

    // Explicit label: screenshot fixture is not evidence of a hardware run.
    await page.evaluate(() => {
      const label = document.createElement('div');
      label.id = 'testWatermark';
      label.textContent = '界面验证 · 模拟遥测 · 未连接硬件';
      label.style.cssText = 'position:fixed;bottom:12px;right:12px;padding:8px 12px;background:#fff4d9;color:#735012;border:1px solid #ddc383;border-radius:4px;font:12px sans-serif;z-index:30';
      document.body.appendChild(label);
    });
    await emit(online());
    await page.screenshot({ path: path.join(output, 'desktop-simulated.png'), fullPage: true });
    await emit({ ...online(), updated_at: Date.now() / 1000 - 5 });
    check(await page.locator('#liveText').textContent() === '数据已过期', 'stale data explicit');
    check(await page.locator('#rateValue').textContent() === '—', 'stale rate not advertised as live');
    check((await page.locator('#detailStatus').textContent()).includes('最后一次接收'), 'stale numeric values explained');
    await emit({ ...online(), sensor: 'error' });
    check((await page.locator('#sensorChip').textContent()).includes('异常'), 'sensor fault indication');
    await emit(online());
    await page.evaluate(() => window.testStream.onerror());
    await page.waitForTimeout(60);
    check((await page.locator('#liveText').textContent()).includes('重连'), 'SSE reconnect explicit');
    await page.evaluate(() => window.testStream.onmessage({ data: 'invalid json' }));
    await page.waitForTimeout(60);
    await emit(online());
    check(await page.locator('#liveText').textContent() === '实时接收中', 'SSE recovers');
    for (const width of [1024, 768, 390, 320]) {
      await page.setViewportSize({ width, height: 1000 });
      await emit(online());
      check(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth),
        `no horizontal overflow at ${width}px`);
      check(await page.locator('#connectButton').isVisible(), `connection available at ${width}px`);
      if (width === 390) await page.screenshot({ path: path.join(output, 'mobile-simulated.png'), fullPage: true });
    }
    await page.locator('#connectButton').click();
    await page.waitForFunction(() => document.querySelector('#liveText').textContent === '设备未连接');
    check(await page.locator('#angleValue').textContent() === '—', 'disconnect clears session readout');
    rejectConnect = true;
    await page.locator('#connectButton').click();
    await page.waitForTimeout(150);
    check((await page.locator('#connectionStatus').textContent()).includes('串口被其他程序占用'),
      'API error survives animation-frame rendering');
    ports = [];
    await page.locator('#refreshButton').click();
    await page.waitForTimeout(80);
    check(await page.locator('#connectButton').isDisabled(), 'no-port connect disabled');
    ports = [{ device: 'COM32', description: '<script>unsafe description</script>' }];
    await page.locator('#refreshButton').click();
    await page.waitForTimeout(80);
    check((await page.locator('#portSelect option').textContent()).includes('<script>'), 'port names rendered as text');
    check(await page.locator('#portSelect script').count() === 0, 'no HTML injection');
    await page.keyboard.press('Tab');
    const focus = await page.evaluate(() => {
      const el = document.activeElement;
      return { outline: getComputedStyle(el).outlineStyle, width: getComputedStyle(el).outlineWidth };
    });
    check(focus.outline !== 'none' && focus.width !== '0px', 'visible keyboard focus');
    check(requests.some((r) => r.endpoint === '/api/calibrate'), 'calibration uses same web serial owner');
    check(controlRequests('parameters').length === 1, 'only accepted tuning confirmation sends parameters');
    check(controlRequests('nn').length === 2, 'only accepted NN confirmations send mode changes');
    check(errors.length === 0, `no JS errors: ${errors.join('; ')}`);
    fs.writeFileSync(path.join(output, 'results.json'), JSON.stringify({
      checks: count, passed: true, hardware: false, javascriptErrors: errors,
      note: 'All telemetry in browser tests is mocked; no serial port is opened.',
    }, null, 2));
    console.log(`PASS ${count} browser checks; screenshots in ${output}. No hardware accessed.`);
  } finally {
    if (browser) await browser.close();
    if (server) await new Promise((resolve) => server.close(resolve));
  }
}
main().catch((error) => { console.error(error); process.exitCode = 1; });
