#!/usr/bin/env node
// End-to-end browser check for the Go2 simulation console.
// Requires the console to be running and (for EXPECT_LIVE=1) a live ROS stack.
import { spawn } from 'node:child_process';
import { mkdirSync, mkdtempSync, writeFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { setTimeout as sleep } from 'node:timers/promises';
import WebSocket from 'ws';

const consoleUrl = process.env.CONSOLE_URL || 'http://127.0.0.1:8765/';
const chromeBinary = process.env.CHROME || 'google-chrome';
const port = Number(process.env.CDP_PORT || 9333);
const live = process.env.EXPECT_LIVE === '1';
const expectFocus = process.env.EXPECT_FOCUS !== '0';
const outDir = process.env.OUT_DIR || '/tmp/go2-web-console-browser';
const profile = mkdtempSync(join(tmpdir(), 'go2-console-chrome-'));
mkdirSync(outDir, {recursive: true});

function getJSON(path) {
  return fetch(`http://127.0.0.1:${port}${path}`).then(response => response.json());
}

async function waitForChrome(timeoutMs = 20000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    try {
      return await getJSON('/json/version');
    } catch {
      await sleep(250);
    }
  }
  throw new Error('Chrome DevTools did not start');
}

function connect(url) {
  return new Promise((resolve, reject) => {
    const socket = new WebSocket(url);
    socket.once('open', () => resolve(socket));
    socket.once('error', reject);
  });
}

async function main() {
  const chrome = spawn(chromeBinary, [
    '--headless=new', '--no-sandbox', '--disable-gpu', '--hide-scrollbars',
    '--remote-debugging-address=127.0.0.1', `--remote-debugging-port=${port}`,
    `--user-data-dir=${profile}`, '--window-size=1440,1000', 'about:blank',
  ], {stdio: 'ignore'});

  let socket;
  try {
    await waitForChrome();
    const page = (await getJSON('/json/list')).find(target => target.type === 'page');
    if (!page) throw new Error('No page target');
    socket = await connect(page.webSocketDebuggerUrl);

    let nextId = 0;
    const pending = new Map();
    const pageErrors = [];
    const consoleMessages = [];
    socket.on('message', raw => {
      const message = JSON.parse(raw);
      if (message.id && pending.has(message.id)) {
        const {resolve, reject} = pending.get(message.id);
        pending.delete(message.id);
        message.error ? reject(new Error(JSON.stringify(message.error))) : resolve(message.result);
      } else if (message.method === 'Runtime.exceptionThrown') {
        pageErrors.push(message.params.exceptionDetails.text);
      } else if (message.method === 'Log.entryAdded' && message.params.entry.level === 'error') {
        pageErrors.push(message.params.entry.text);
      } else if (message.method === 'Runtime.consoleAPICalled') {
        consoleMessages.push(message.params.args.map(arg => arg.value ?? arg.description).join(' '));
      }
    });
    const send = (method, params = {}) => new Promise((resolve, reject) => {
      const id = ++nextId;
      pending.set(id, {resolve, reject});
      socket.send(JSON.stringify({id, method, params}));
    });
    const evaluate = async expression => {
      const result = await send('Runtime.evaluate', {expression, returnByValue: true, awaitPromise: true});
      if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
      return result.result?.value;
    };
    const screenshot = async name => {
      const result = await send('Page.captureScreenshot', {format: 'png'});
      writeFileSync(join(outDir, name), Buffer.from(result.data, 'base64'));
    };

    await send('Page.enable');
    await send('Runtime.enable');
    await send('Log.enable');
    await send('Page.navigate', {url: consoleUrl});
    await sleep(10000);

    const dom = await evaluate(`(() => {
      const v = id => document.getElementById(id)?.textContent?.trim();
      const canvas = document.querySelector('#vnc-screen canvas');
      let pixels = null;
      if (canvas) {
        const data = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
        let nonBlank = 0;
        for (let i = 0; i < data.length; i += 4) if (data[i] || data[i+1] || data[i+2]) nonBlank++;
        pixels = {w: canvas.width, h: canvas.height, nonBlank};
      }
      return {
        title: document.title, stream: v('stream-state'), mission: v('mission-state'),
        domain: v('domain-value'), battery: v('battery-value'), linear: v('linear-value'),
        clearance: v('clearance-value'), guard: v('guard-reason'), poseAge: v('pose-age'),
        mapDetails: v('map-details'), windowLabel: v('window-label'), vnc: pixels,
        interaction: {
          text: document.getElementById('interaction')?.textContent,
          pressed: document.getElementById('interaction')?.getAttribute('aria-pressed'),
        },
      };
    })()`);
    await screenshot('desktop.png');

    const checks = {
      title: /Go2/.test(dom.title || ''),
      stream: dom.stream === '画面已连接',
      vncPixels: Boolean(dom.vnc && dom.vnc.nonBlank > 1000),
      liveTelemetry: !live || (
        Boolean(dom.domain) && dom.domain !== '—' &&
        Boolean(dom.mission) && dom.mission !== '未接入' && dom.mission !== '状态连接已断开' &&
        /%/.test(dom.battery || '') &&
        /m\/px/.test(dom.mapDetails || '') &&
        Boolean(dom.poseAge) && dom.poseAge !== '未接入'
      ),
    };

    await evaluate(`document.getElementById('interaction').click()`);
    await sleep(600);
    const off = await evaluate(`({
      text: document.getElementById('interaction').textContent,
      pressed: document.getElementById('interaction').getAttribute('aria-pressed'),
    })`);
    await evaluate(`document.getElementById('interaction').click()`);
    await sleep(600);
    const on = await evaluate(`({
      text: document.getElementById('interaction').textContent,
      pressed: document.getElementById('interaction').getAttribute('aria-pressed'),
    })`);
    checks.interactionToggle = off.pressed === 'false' && on.pressed === 'true';

    let focusToast = null;
    if (live) {
      await evaluate(`document.getElementById('focus-robot').click()`);
      await sleep(4000);
      focusToast = await evaluate(`({
        text: document.getElementById('toast').textContent,
        hidden: document.getElementById('toast').hidden,
      })`);
      if (expectFocus) checks.focusRobot = (focusToast.text || '').includes('镜头已移向机器狗');
    }

    await evaluate(`document.getElementById('reconnect').click()`);
    await sleep(6000);
    checks.reconnect = await evaluate(`document.getElementById('stream-state').textContent === '画面已连接'`);

    await send('Emulation.setDeviceMetricsOverride', {width: 390, height: 844, deviceScaleFactor: 2, mobile: true});
    await sleep(2500);
    await screenshot('mobile.png');
    const mobile = await evaluate(`({
      width: innerWidth, scrollWidth: document.documentElement.scrollWidth,
    })`);
    checks.mobile = mobile.scrollWidth <= mobile.width + 1;
    checks.pageErrors = pageErrors.length === 0;

    const result = {success: Object.values(checks).every(Boolean), checks, dom, off, on, focusToast, mobile, pageErrors, consoleMessages};
    writeFileSync(join(outDir, 'result.json'), JSON.stringify(result, null, 2));
    console.log(JSON.stringify(result, null, 2));
    return result.success ? 0 : 1;
  } finally {
    if (socket) socket.close();
    chrome.kill('SIGTERM');
    await sleep(500);
    rmSync(profile, {recursive: true, force: true});
  }
}

main().then(code => process.exit(code)).catch(error => {
  console.error(error);
  process.exit(1);
});
