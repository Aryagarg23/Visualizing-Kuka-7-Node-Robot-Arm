import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { decodeLinks, decodeRuns, row } from '../src/data.js';

const load = p => { const b = readFileSync(new URL(p, import.meta.url)); return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength); };
const links = decodeLinks(load('../public/data/iiwa7-links.bin'));
const runs = decodeRuns(load('../public/data/kuka-trajectories.bin'));
const summary = JSON.parse(readFileSync(new URL('./fixtures/links-summary.json', import.meta.url)));
const rows = JSON.parse(readFileSync(new URL('./fixtures/kuka-rows.json', import.meta.url)));

test('every link mesh decodes to the counts and bounds the build wrote', () => {
  const names = Object.keys(summary);
  assert.equal(links.length, names.length);
  names.forEach((name, i) => {
    const { positions, indices } = links[i];
    assert.equal(positions.length / 3, summary[name].vertices, name);
    assert.equal(indices.length / 3, summary[name].faces, name);
    for (const k of indices) assert.ok(k < positions.length / 3, `${name} index in range`);
    for (let a = 0; a < 3; a++) {
      let lo = Infinity, hi = -Infinity;
      for (let v = a; v < positions.length; v += 3) { lo = Math.min(lo, positions[v]); hi = Math.max(hi, positions[v]); }
      assert.ok(Math.abs(lo - summary[name].min[a]) < 1e-3 && Math.abs(hi - summary[name].max[a]) < 1e-3, `${name} axis ${a}`);
    }
  });
});

test('the stray A7 sliver at joint-5 height is gone, and only it', () => {
  const a7 = summary.A7;
  assert.equal(a7.sourceFaces - a7.faces, 4);
  assert.ok(a7.min[2] > 1200, `A7 starts at ${a7.min[2]} mm`);
  for (const name of Object.keys(summary)) if (name !== 'A7') assert.equal(summary[name].faces, summary[name].sourceFaces, name);
});

test('the recordings decode to the .mat values: positions, derived velocities, torques', () => {
  assert.deepEqual(runs.map(r => r.length), rows.lengths);
  let worst = { pos: 0, vel: 0, tau: 0 };
  for (const [run, { index, data }] of Object.entries(rows.rows)) {
    const r = runs[Number(run) - 1];
    index.forEach((i, k) => {
      const src = data[k];
      const p = row(r.position, i), v = row(r.velocity, i), t = row(r.torque, i);
      for (let j = 0; j < 7; j++) {
        worst.pos = Math.max(worst.pos, Math.abs(p[j] - src[j]));
        worst.vel = Math.max(worst.vel, Math.abs(v[j] - src[7 + j]));
        worst.tau = Math.max(worst.tau, Math.abs(t[j] - src[14 + j]));
      }
    });
  }
  assert.ok(worst.pos < 1e-6, `position ${worst.pos} rad`);
  assert.ok(worst.vel < 1e-4, `velocity ${worst.vel} rad/s`);
  assert.ok(worst.tau < 1e-5, `torque ${worst.tau} N·m`);
});

test('recorded at 100 Hz, about 20 s a run', () => {
  for (const r of runs) {
    assert.equal(r.hz, 100);
    assert.ok(r.length / r.hz > 17 && r.length / r.hz < 26, `${r.length / r.hz} s`);
  }
});
