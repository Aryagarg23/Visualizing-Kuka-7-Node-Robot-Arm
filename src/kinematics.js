// LBR iiwa 7 R800 kinematics in millimetres, z up, base at the origin.
//
// The meshes are modelled with every joint at zero (the arm straight up), so
// each joint is an axis through a point on the centre line, and a link's pose
// is the product of the rotations of every joint below it, each about its own
// zero-pose axis. That is the "rotation relative to the node below it" chain
// the Unity build hand-wrote rotation scripts for.
//
// Axis directions and signs follow KUKA's URDF (iiwa_description): joints 1,
// 3, 5, 7 turn about +z; 2 and 6 about +y; 4 about -y. With those signs the
// recorded pick-and-place keeps the tool pointing at the table for the whole
// run (test/kinematics.test.mjs pins this), which is how a pick-and-place is
// done; flip joint 4 and the tool waves about instead.

export const JOINTS = [
  { axis: [0, 0, 1], at: 157.5, limit: 170 },
  { axis: [0, 1, 0], at: 340, limit: 120 },
  { axis: [0, 0, 1], at: 557.5, limit: 170 },
  { axis: [0, -1, 0], at: 740, limit: 120 },
  { axis: [0, 0, 1], at: 957.5, limit: 170 },
  { axis: [0, 1, 0], at: 1140, limit: 120 },
  { axis: [0, 0, 1], at: 1220.4, limit: 175 },
];
export const FLANGE_Z = 1266;
export const DEG = Math.PI / 180;
export const LIMITS = JOINTS.map(j => j.limit * DEG);

// 4x4 matrices, column-major (the order three.js Matrix4.fromArray reads).
const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

function multiply(a, b) {
  const out = new Array(16);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      out[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
    }
  }
  return out;
}

// Rotation by `angle` about the line through (0, 0, z) along unit `axis`.
function rotationAbout(axis, z, angle) {
  const [x, y, w] = axis;
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  const r = [
    t * x * x + c, t * x * y + s * w, t * x * w - s * y, 0,
    t * x * y - s * w, t * y * y + c, t * y * w + s * x, 0,
    t * x * w + s * y, t * y * w - s * x, t * w * w + c, 0,
    0, 0, 0, 1,
  ];
  // translate(0,0,z) · r · translate(0,0,-z)
  r[12] = -r[8] * z;
  r[13] = -r[9] * z;
  r[14] = z - r[10] * z;
  return r;
}

export const apply = (m, [x, y, z]) => [
  m[0] * x + m[4] * y + m[8] * z + m[12],
  m[1] * x + m[5] * y + m[9] * z + m[13],
  m[2] * x + m[6] * y + m[10] * z + m[14],
];
export const applyDir = (m, [x, y, z]) => [
  m[0] * x + m[4] * y + m[8] * z,
  m[1] * x + m[5] * y + m[9] * z,
  m[2] * x + m[6] * y + m[10] * z,
];

/** World matrices for the base and links 1..7 (8 in all) at joint angles q (radians). */
export function linkMatrices(q, joints = JOINTS) {
  const out = [identity()];
  let m = out[0];
  for (let i = 0; i < 7; i++) {
    m = multiply(m, rotationAbout(joints[i].axis, joints[i].at, q[i]));
    out.push(m);
  }
  return out;
}

/** Flange position (mm), tool axis (unit, the flange's z) and per-joint world axes and points. */
export function forward(q, joints = JOINTS) {
  const ms = linkMatrices(q, joints);
  const tip = ms[7];
  return {
    matrices: ms,
    position: apply(tip, [0, 0, FLANGE_Z]),
    toolAxis: applyDir(tip, [0, 0, 1]),
    // Joint i turns about its axis as carried by everything below it.
    axes: joints.map((j, i) => applyDir(ms[i], j.axis)),
    points: joints.map((j, i) => apply(ms[i], [0, 0, j.at])),
  };
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = a => Math.hypot(a[0], a[1], a[2]);

export const clampToLimits = q => q.map((v, i) => Math.max(-LIMITS[i], Math.min(LIMITS[i], v)));

// Solve the n x n system A x = b (Gaussian elimination, partial pivoting).
function solve(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

// Weighting: one radian of tool tilt counts as much as this many millimetres
// of position error.
const TILT_MM = 400;
const DAMPING = 8; // damped least squares, mm
const MAX_STEP_MM = 60; // per iteration, so a far target is approached, not leapt at

/**
 * Inverse kinematics: joint angles that put the flange at `target` (mm) with
 * the tool pointing along `toolDir` (default straight down, as in the
 * recordings), starting from `q0`.
 *
 * Damped least squares on the 7 joints: 3 rows for position and 2 for tool
 * tilt, since turning about the tool's own axis does not matter here. That
 * leaves two spare joints, and starting from the current pose and taking
 * the smallest step that closes the error is what keeps the arm from
 * jumping to another of the many poses that reach the same point. The
 * article's planar sketch was about exactly that choice.
 */
export function solveIk(target, q0, { toolDir = [0, 0, -1], iterations = 200, tolerance = 0.05 } = {}) {
  let q = clampToLimits(q0);
  let best = null;
  for (let it = 0; it < iterations; it++) {
    const f = forward(q);
    let ep = sub(target, f.position);
    const ePos = norm(ep);
    // Rotation that would turn the tool axis onto toolDir; its component
    // about the tool axis is zero by construction.
    const eRot = cross(f.toolAxis, toolDir);
    const tilt = Math.atan2(norm(eRot), dot(f.toolAxis, toolDir));
    const score = ePos + tilt * TILT_MM;
    if (!best || score < best.score) best = { q, ePos, tilt, score, iterations: it };
    if (ePos < tolerance && tilt < tolerance / TILT_MM) break;
    if (ePos > MAX_STEP_MM) ep = ep.map(v => (v * MAX_STEP_MM) / ePos);
    const rotLen = norm(eRot);
    const eo = rotLen > 1e-12 ? eRot.map(v => (v / rotLen) * tilt * TILT_MM) : [0, 0, 0];

    // Jacobian rows: position (3) and tilt (the rotation's components across
    // the tool axis, written in a basis u, v perpendicular to it).
    const t = f.toolAxis;
    const u = norm(cross(t, [1, 0, 0])) > 0.1 ? cross(t, [1, 0, 0]) : cross(t, [0, 1, 0]);
    const uN = u.map(v => v / norm(u));
    const vN = cross(t, uN);
    const J = [[], [], [], [], []];
    for (let i = 0; i < 7; i++) {
      const lin = cross(f.axes[i], sub(f.position, f.points[i]));
      J[0].push(lin[0]); J[1].push(lin[1]); J[2].push(lin[2]);
      J[3].push(dot(f.axes[i], uN) * TILT_MM);
      J[4].push(dot(f.axes[i], vN) * TILT_MM);
    }
    const e = [ep[0], ep[1], ep[2], dot(eo, uN), dot(eo, vN)];
    // dq = J^T (J J^T + λ² I)^-1 e
    const JJt = J.map((ri, a) => J.map((rj, b) => ri.reduce((s, v, k) => s + v * rj[k], 0) + (a === b ? DAMPING * DAMPING : 0)));
    const y = solve(JJt, e);
    const dq = q.map((_, k) => J.reduce((s, row, a) => s + row[k] * y[a], 0));
    q = clampToLimits(q.map((v, k) => v + dq[k]));
  }
  const reached = best.ePos < 0.5 && best.tilt < 0.5 * DEG;
  return { q: best.q, reached, positionError: best.ePos, tiltError: best.tilt, iterations: best.iterations };
}
