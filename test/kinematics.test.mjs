import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { DEG, FLANGE_Z, JOINTS, LIMITS, forward, solveIk } from '../src/kinematics.js';
import { decodeRuns, row } from '../src/data.js';

const load = p => { const b = readFileSync(new URL(p, import.meta.url)); return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength); };
const runs = decodeRuns(load('../public/data/kuka-trajectories.bin'));
const summary = JSON.parse(readFileSync(new URL('./fixtures/links-summary.json', import.meta.url)));
const close = (a, b, eps) => a.every((v, i) => Math.abs(v - b[i]) < eps);

test('all joints at zero: the arm stands straight up, flange at 1266 mm', () => {
  const f = forward([0, 0, 0, 0, 0, 0, 0]);
  assert.ok(close(f.position, [0, 0, FLANGE_Z], 1e-9));
  assert.ok(close(f.toolAxis, [0, 0, 1], 1e-12));
});

test('each joint sits where its two link meshes meet', () => {
  const names = Object.keys(summary); // Base, A1..A7
  JOINTS.forEach((j, i) => {
    const below = summary[names[i]], above = summary[names[i + 1]];
    assert.ok(j.at >= above.min[2] - 6 && j.at <= below.max[2] + 1, `joint ${i + 1} at ${j.at}: ${above.min[2]}..${below.max[2]}`);
  });
});

// Why joint 4 turns about -y: with KUKA's signs the recorded pick-and-place
// holds the tool near straight down for most of every run (run 8 leans it
// up to 51° for a stretch). The same data played with joint 4 flipped points
// the tool up and away from the table the whole time.
function medianTilts(joints) {
  return runs.map(r => {
    const tilts = [];
    for (let i = 0; i < r.length; i += 5) {
      const t = forward(row(r.position, i), joints).toolAxis;
      tilts.push(Math.acos(Math.max(-1, Math.min(1, -t[2]))) / DEG);
    }
    return tilts.sort((a, b) => a - b)[tilts.length >> 1];
  });
}

test('the recordings keep the tool pointing down (joint signs are right)', () => {
  const right = medianTilts();
  const flipped = medianTilts(JOINTS.map((j, i) => (i === 3 ? { ...j, axis: [0, 1, 0] } : j)));
  console.log(`  median tilt per run ${right.map(v => v.toFixed(0)).join(' ')}°; joint 4 flipped ${flipped.map(v => v.toFixed(0)).join(' ')}°`);
  for (const v of right) assert.ok(v < 20, `${v}°`);
  for (const v of flipped) assert.ok(v > 140, `${v}°`);
});

test('every recorded angle is inside the iiwa joint limits', () => {
  for (const r of runs) for (let i = 0; i < r.position.length; i++) assert.ok(Math.abs(r.position[i]) <= LIMITS[i % 7]);
});

const home = row(runs[0].position, 0);

test('IK reaches every 25th recorded flange position, tool down, inside the limits', () => {
  let n = 0, worstPos = 0, worstTilt = 0;
  for (const r of runs) {
    for (let i = 0; i < r.length; i += 25) {
      const target = forward(row(r.position, i)).position;
      const s = solveIk(target, home);
      assert.ok(s.reached, `run sample ${i}: ${s.positionError.toFixed(2)} mm, ${(s.tiltError / DEG).toFixed(2)}°`);
      s.q.forEach((v, k) => assert.ok(Math.abs(v) <= LIMITS[k] + 1e-12));
      worstPos = Math.max(worstPos, s.positionError);
      worstTilt = Math.max(worstTilt, s.tiltError);
      n++;
    }
  }
  console.log(`  ${n} targets, worst ${worstPos.toFixed(3)} mm, ${(worstTilt / DEG).toFixed(3)}°`);
});

// Why the solver starts from the current pose: following run 1's flange path
// at the recorded 100 Hz, each solution stays next to the last, so the drawn
// arm moves smoothly instead of hopping between poses that reach the same
// point.
test('following a recorded path, IK moves the joints as little as the robot did', () => {
  const r = runs[0];
  let q = home, worstStep = 0, recordedWorst = 0;
  for (let i = 1; i < r.length; i++) {
    const target = forward(row(r.position, i)).position;
    const s = solveIk(target, q);
    assert.ok(s.reached);
    worstStep = Math.max(worstStep, ...s.q.map((v, k) => Math.abs(v - q[k])));
    recordedWorst = Math.max(recordedWorst, ...row(r.velocity, i).map(v => Math.abs(v) / r.hz));
    q = s.q;
  }
  console.log(`  worst joint step ${(worstStep / DEG).toFixed(3)}°, recorded ${(recordedWorst / DEG).toFixed(3)}°`);
  assert.ok(worstStep < 3 * recordedWorst, `${worstStep / DEG}° vs ${recordedWorst / DEG}°`);
});

test('a target out of reach is reported, and the pose stays inside the limits', () => {
  const s = solveIk([1500, 0, 600], home);
  assert.equal(s.reached, false);
  s.q.forEach((v, k) => assert.ok(Math.abs(v) <= LIMITS[k] + 1e-12));
  assert.ok(s.positionError > 100);
});

// Why the solver handles joint limits, not just clamps to them: dragging the
// target, the arm would end a few millimetres short of points it can reach,
// with joint 6 pinned at 120°. Starting from recorded poses with joint 2, 4
// or 6 forced onto a limit, the clamping solver missed 149 of these 2442.
// Locking the pinned joint and easing joints off their limits gets most of
// them; the rest need a restart from another pose (counted below).
test('starting with a joint pinned on its limit, IK still reaches the recorded positions', () => {
  let n = 0, restarted = 0;
  for (const r of runs) {
    for (let i = 0; i < r.length; i += 50) {
      const q = row(r.position, i);
      const target = forward(q).position;
      for (const k of [1, 3, 5]) {
        for (const side of [1, -1]) {
          const seed = q.slice();
          seed[k] = side * LIMITS[k];
          const s = solveIk(target, seed);
          assert.ok(s.reached, `run sample ${i}, joint ${k + 1} at ${side > 0 ? '+' : '-'}limit: ${s.positionError.toFixed(2)} mm short`);
          if (s.restarted) restarted++;
          n++;
        }
      }
    }
  }
  console.log(`  ${n} pinned starts, ${restarted} needed a restart`);
});
