import { DEG, JOINTS, LIMITS, forward, solveIk } from './kinematics.js';
import { decodeLinks, decodeRuns, peaks, row } from './data.js';
import { createScene, palette } from './scene.js';

const $ = id => document.getElementById(id);
const canvas = $('view');
const fmt = (v, d = 1) => (v < 0 ? '−' : ' ') + Math.abs(v).toFixed(d);

async function fetchBuffer(name) {
  const res = await fetch(new URL(`data/${name}`, document.baseURI));
  if (!res.ok) throw new Error(`${name}: HTTP ${res.status}`);
  return res.arrayBuffer();
}

const [links, runs] = await Promise.all([
  fetchBuffer('iiwa7-links.bin').then(decodeLinks),
  fetchBuffer('kuka-trajectories.bin').then(decodeRuns),
]).catch(e => { $('loading').textContent = `Could not load the arm (${e.message}).`; throw e; });
$('loading').remove();

const scene = createScene(canvas, links);
const torquePeak = peaks(runs, 'torque');
const reduced = window.matchMedia?.('(prefers-reduced-motion: reduce)').matches;

const state = {
  mode: 'recorded',
  run: 0,
  t: 0, // sample index, fractional while playing
  playing: !reduced,
  speed: 1,
  q: row(runs[0].position, 0),
  target: null,
  reach: null,
};

// Flange path per run, every 4th sample, computed once on first view.
const paths = new Map();
function pathFor(r) {
  if (!paths.has(r)) {
    const run = runs[r], step = 4, pts = new Float32Array(Math.ceil(run.length / step) * 3);
    for (let i = 0, k = 0; i < run.length; i += step, k += 3) pts.set(forward(row(run.position, i)).position, k);
    paths.set(r, pts);
  }
  return paths.get(r);
}

// ----- joint table -------------------------------------------------------
const rows = JOINTS.map((j, i) => {
  const tr = document.createElement('tr');
  tr.innerHTML = `<td>J${i + 1}</td>
    <td><input type="range" min="${-j.limit}" max="${j.limit}" step="0.1" aria-label="Joint ${i + 1} angle in degrees"></td>
    <td class="num angle"></td><td class="num vel"></td>
    <td class="num torque"><div class="bar"></div><span></span></td>`;
  $('joint-rows').append(tr);
  const slider = tr.querySelector('input');
  slider.addEventListener('input', () => {
    if (state.mode !== 'joints') setMode('joints');
    state.q[i] = Number(slider.value) * DEG;
    update();
  });
  return { slider, angle: tr.querySelector('.angle'), vel: tr.querySelector('.vel'), bar: tr.querySelector('.bar'), torque: tr.querySelector('.torque span') };
});

// ----- torque chart ------------------------------------------------------
const chart = $('torque');
let chartCache = null;
function drawChart() {
  const run = runs[state.run], dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = chart.clientWidth, h = chart.clientHeight;
  if (!w) return;
  const colors = palette();
  if (!chartCache || chartCache.run !== state.run || chartCache.w !== w) {
    const off = document.createElement('canvas');
    off.width = w * dpr; off.height = h * dpr;
    const c = off.getContext('2d');
    c.scale(dpr, dpr);
    const lane = h / 7;
    c.font = '10px ui-monospace, Menlo, monospace';
    for (let j = 0; j < 7; j++) {
      const mid = lane * j + lane / 2;
      c.strokeStyle = colors.rule; c.lineWidth = 0.5;
      c.beginPath(); c.moveTo(0, mid); c.lineTo(w, mid); c.stroke();
      c.strokeStyle = colors.ink; c.lineWidth = 1;
      c.beginPath();
      for (let x = 0; x < w; x++) {
        const i = Math.floor((x / w) * run.length);
        const y = mid - (run.torque[i * 7 + j] / torquePeak[j]) * (lane / 2 - 1.5);
        x ? c.lineTo(x, y) : c.moveTo(x, y);
      }
      c.stroke();
      c.fillStyle = colors.ink;
      c.fillText(`J${j + 1}`, 3, lane * j + 10);
    }
    chartCache = { run: state.run, w, canvas: off };
  }
  chart.width = w * dpr; chart.height = h * dpr;
  const c = chart.getContext('2d');
  c.drawImage(chartCache.canvas, 0, 0);
  const x = (state.t / run.length) * w * dpr;
  c.strokeStyle = colors.hot; c.lineWidth = 2 * dpr;
  c.beginPath(); c.moveTo(x, 0); c.lineTo(x, h * dpr); c.stroke();
}
chart.addEventListener('pointerdown', e => {
  const r = chart.getBoundingClientRect();
  seek(((e.clientX - r.left) / r.width) * runs[state.run].length);
});

// ----- recorded playback --------------------------------------------------
runs.forEach((r, i) => $('run').append(new Option(`${i + 1} (${(r.length / r.hz).toFixed(1)} s)`, i)));
$('run').addEventListener('change', e => { state.run = Number(e.target.value); seek(0); });
$('speed').addEventListener('change', e => { state.speed = Number(e.target.value); });
$('play').addEventListener('click', () => { setPlaying(!state.playing); });
$('scrub').addEventListener('input', e => { setPlaying(false); seek(Number(e.target.value)); });

function setPlaying(on) {
  state.playing = on;
  $('play').textContent = on ? 'Pause' : 'Play';
}
function seek(t) {
  const run = runs[state.run];
  state.t = Math.max(0, Math.min(run.length - 1, t));
  if (state.mode !== 'recorded') setMode('recorded');
  update();
}

function recordedSample() {
  const run = runs[state.run], i = Math.floor(state.t), k = Math.min(i + 1, run.length - 1), f = state.t - i;
  const lerp = ch => row(ch, i).map((v, j) => v + (ch[k * 7 + j] - v) * f);
  return { q: lerp(run.position), vel: lerp(run.velocity), torque: lerp(run.torque) };
}

// ----- modes --------------------------------------------------------------
document.querySelectorAll('[data-mode]').forEach(b => b.addEventListener('click', () => setMode(b.dataset.mode)));
$('zero').addEventListener('click', () => { state.q = [0, 0, 0, 0, 0, 0, 0]; update(); });
$('from-run').addEventListener('click', () => { state.q = recordedSample().q; update(); });

function setMode(mode) {
  state.mode = mode;
  document.querySelectorAll('[data-mode]').forEach(b => b.setAttribute('aria-selected', String(b.dataset.mode === mode)));
  document.querySelectorAll('.mode-body').forEach(el => { el.hidden = el.dataset.for !== mode; });
  $('torque-figure').hidden = mode !== 'recorded';
  if (mode !== 'recorded') setPlaying(false);
  if (mode === 'reach') {
    const p = forward(state.q).position;
    state.target = p;
    state.reach = null;
    canvas.focus();
  } else state.target = null;
  update();
}

// ----- reach --------------------------------------------------------------
$('target-z').addEventListener('input', e => moveTarget([state.target[0], state.target[1], Number(e.target.value)]));
function moveTarget(p) {
  state.target = [p[0], p[1], Math.max(50, Math.min(1200, p[2]))];
  state.reach = solveIk(state.target, state.q);
  state.q = state.reach.q;
  update();
}

canvas.tabIndex = 0;
canvas.addEventListener('keydown', e => {
  if (state.mode !== 'reach') return;
  const s = e.shiftKey ? 50 : 10, t = state.target;
  const moves = { ArrowLeft: [-s, 0, 0], ArrowRight: [s, 0, 0], ArrowUp: [0, s, 0], ArrowDown: [0, -s, 0], PageUp: [0, 0, s], PageDown: [0, 0, -s] };
  if (!moves[e.key]) return;
  e.preventDefault();
  // Arrow keys move along the screen's left/right and up/down, turned onto the floor.
  const [dx, dy, dz] = moves[e.key], a = scene.view.azimuth;
  const right = [-Math.sin(a), Math.cos(a)], away = [-Math.cos(a), -Math.sin(a)];
  moveTarget([t[0] + dx * right[0] + dy * away[0], t[1] + dx * right[1] + dy * away[1], t[2] + dz]);
});

// ----- view: orbit, zoom, drag the target ----------------------------------
let drag = null;
const overTarget = (x, y) => {
  if (state.mode !== 'reach' || !state.target) return false;
  const [sx, sy] = scene.toScreen(state.target);
  return Math.hypot(sx - x, sy - y) < 30;
};
canvas.addEventListener('pointerdown', e => {
  const r = canvas.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  drag = overTarget(x, y) ? { kind: 'target' } : { kind: 'orbit', x: e.clientX, y: e.clientY };
  canvas.setPointerCapture(e.pointerId);
  canvas.classList.toggle('dragging', drag.kind === 'orbit');
});
canvas.addEventListener('pointermove', e => {
  const r = canvas.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  if (!drag) { canvas.classList.toggle('over-target', overTarget(x, y)); return; }
  if (drag.kind === 'target') {
    const hit = scene.onPlane(x, y, state.target[2]);
    if (hit) moveTarget([hit[0], hit[1], state.target[2]]);
  } else {
    scene.view.azimuth -= (e.clientX - drag.x) * 0.008;
    scene.view.elevation = Math.max(0.05, Math.min(1.45, scene.view.elevation + (e.clientY - drag.y) * 0.006));
    drag.x = e.clientX; drag.y = e.clientY;
    scene.invalidate();
  }
});
const endDrag = () => { drag = null; canvas.classList.remove('dragging'); };
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  scene.view.zoom = Math.max(0.5, Math.min(4, scene.view.zoom * Math.exp(-e.deltaY * 0.0015)));
  scene.invalidate();
}, { passive: false });
canvas.addEventListener('dblclick', () => scene.resetView());
$('turn-left').addEventListener('click', () => { scene.view.azimuth += Math.PI / 2; scene.invalidate(); });
$('turn-right').addEventListener('click', () => { scene.view.azimuth -= Math.PI / 2; scene.invalidate(); });
$('iso').addEventListener('click', () => scene.resetView());
new ResizeObserver(() => { chartCache = null; scene.invalidate(); drawChart(); }).observe(document.body);

// ----- one update for every change -----------------------------------------
function update() {
  const run = runs[state.run];
  let vel = null, torque = null;
  if (state.mode === 'recorded') ({ q: state.q, vel, torque } = recordedSample());
  const f = forward(state.q);

  scene.setPose(state.q, torque && torque.map((v, j) => Math.abs(v) / torquePeak[j]));
  scene.setFlange(f.position);
  scene.setPath(state.mode === 'recorded' ? pathFor(state.run) : null);
  scene.setPathProgress(Math.floor(state.t / 4) + 1);
  scene.setTarget(state.mode === 'reach' ? state.target : null);

  rows.forEach((r, j) => {
    const deg = state.q[j] / DEG;
    if (document.activeElement !== r.slider) r.slider.value = deg.toFixed(1);
    r.angle.textContent = fmt(deg);
    r.vel.textContent = vel ? fmt(vel[j] / DEG) : '—';
    r.torque.textContent = torque ? fmt(torque[j], 2) : '—';
    r.bar.style.width = torque ? `${Math.min(100, (Math.abs(torque[j]) / torquePeak[j]) * 100)}%` : '0';
  });

  const [x, y, z] = f.position;
  const tilt = Math.acos(Math.max(-1, Math.min(1, -f.toolAxis[2]))) / DEG;
  $('readout').textContent = `FLANGE  x ${fmt(x)}  y ${fmt(y)}  z ${fmt(z)} mm\nTOOL    ${tilt.toFixed(1)}° from straight down`;

  if (state.mode === 'recorded') {
    $('scrub').max = String(run.length - 1);
    $('scrub').value = String(Math.round(state.t));
    $('clock').textContent = `${(state.t / run.hz).toFixed(2)} / ${(run.length / run.hz).toFixed(2)} s`;
    $('note').textContent = 'Link shading: the torque at that link\'s joint against the largest at that joint in all ten runs.';
    drawChart();
  } else {
    $('note').textContent = 'Torque is only shown for the recordings; this page has no dynamics model, so the links are not shaded here.';
  }
  if (state.mode === 'reach') {
    $('target-z').value = String(Math.round(state.target[2]));
    $('target-z-out').textContent = `z ${state.target[2].toFixed(0)} mm`;
    const s = state.reach, el = $('reach-status');
    el.classList.toggle('miss', !!s && !s.reached);
    el.textContent = !s ? 'Target on the flange. Move it.'
      : s.reached ? `Reached. Tool ${(s.tiltError / DEG).toFixed(2)}° off vertical.`
      : `Out of reach: the flange stops ${s.positionError.toFixed(0)} mm short.`;
  }
}

let last = performance.now();
function tick(now) {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (state.playing && state.mode === 'recorded') {
    const run = runs[state.run];
    state.t += dt * run.hz * state.speed;
    if (state.t >= run.length - 1) state.t = 0;
    update();
  }
  scene.render();
  const handle = $('handle');
  handle.hidden = state.mode !== 'reach';
  if (!handle.hidden) {
    const [x, y] = scene.toScreen(state.target);
    handle.style.transform = `translate(${x}px, ${y}px)`;
  }
  requestAnimationFrame(tick);
}
setPlaying(state.playing);
update();
requestAnimationFrame(tick);
