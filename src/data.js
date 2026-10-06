// Decoders for the two files scripts/build_data.py writes.

/** iiwa7-links.bin → [{ positions: Float32Array (mm), indices: Uint16Array }] for Base, A1..A7. */
export function decodeLinks(buffer) {
  const view = new DataView(buffer);
  if (String.fromCharCode(...new Uint8Array(buffer, 0, 4)) !== 'IIWA') throw new Error('not a link mesh file');
  const count = view.getUint32(4, true);
  const links = [];
  let o = 8;
  for (let i = 0; i < count; i++) {
    const nv = view.getUint32(o, true), nf = view.getUint32(o + 4, true);
    o += 8;
    const positions = new Float32Array(buffer.slice(o, o + nv * 12));
    o += nv * 12;
    const indices = new Uint16Array(buffer.slice(o, o + nf * 6));
    o += nf * 6;
    o = (o + 3) & ~3;
    links.push({ positions, indices });
  }
  return links;
}

/**
 * kuka-trajectories.bin → runs, each { length, hz, position, velocity, torque }
 * where every channel is a Float32Array of length*7, row-major (sample, joint).
 * Positions in radians, velocities in rad/s, torques in N·m.
 */
export function decodeRuns(buffer) {
  const view = new DataView(buffer);
  if (String.fromCharCode(...new Uint8Array(buffer, 0, 4)) !== 'KDYN') throw new Error('not a trajectory file');
  const count = view.getUint32(4, true), hz = view.getUint32(8, true), torquePerNm = view.getUint32(12, true);
  const runs = [];
  let o = 16;
  for (let r = 0; r < count; r++) {
    const length = view.getUint32(o, true);
    o += 4;
    const position = new Float32Array(buffer.slice(o, o + length * 28));
    o += length * 28;
    const steps = new Int16Array(buffer.slice(o, o + length * 14));
    o += length * 14;
    o = (o + 3) & ~3;
    const torque = Float32Array.from(steps, s => s / torquePerNm);
    // The recorded velocity is the position step times the sample rate, zero
    // on the first row (scripts/build_data.py asserts it before dropping it).
    const velocity = new Float32Array(length * 7);
    for (let i = 7; i < length * 7; i++) velocity[i] = (position[i] - position[i - 7]) * hz;
    runs.push({ length, hz, position, velocity, torque });
  }
  return runs;
}

export const row = (channel, i) => Array.from(channel.subarray(i * 7, i * 7 + 7));

/** Largest |value| per joint across every run, for scaling bars and colours. */
export function peaks(runs, key) {
  const out = new Array(7).fill(0);
  for (const run of runs) {
    const c = run[key];
    for (let i = 0; i < c.length; i++) out[i % 7] = Math.max(out[i % 7], Math.abs(c[i]));
  }
  return out;
}
