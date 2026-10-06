"""Turn the hackathon inputs in source/ into the two files the page loads.

  public/data/iiwa7-links.bin       the eight LBR iiwa 7 R800 meshes, in mm
  public/data/kuka-trajectories.bin Kinetic Vision's ten KUKA pick-and-place runs

and a small fixture the tests compare the decoders against. Run it after
changing anything in source/:

    python3 scripts/build_data.py

Needs numpy and scipy (to read the .mat file).
"""

import json
import pathlib
import struct

import numpy as np
import scipy.io as sio

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / "source"
OUT = ROOT / "public" / "data"
FIXTURES = ROOT / "test" / "fixtures"

# Base, then the link each joint turns. The OBJ files are the ones Kinetic
# Vision handed out, exported from Fusion 360 in tenths of a millimetre with
# the arm standing straight up (every joint at zero).
LINKS = ["Base", "A1", "A2", "A3", "A4", "A5", "A6", "A7"]
OBJ_UNIT_MM = 0.1

# A7.obj carries four stray triangles 0.1 mm wide down at the height of joint
# 5 (z = 957.5 to 963 mm), far below the flange they belong to. Turning with
# joint 7 they would float beside the wrist, so faces with any vertex below
# this height are dropped from A7.
A7_MIN_Z_MM = 1200.0

SAMPLE_HZ = 100
TORQUE_STEP_NM = 0.01  # the recordings carry torques to two decimals


def read_obj(path):
    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
            for k in range(1, len(idx) - 1):  # fan any polygon into triangles
                faces.append([idx[0], idx[k], idx[k + 1]])
    return np.array(verts) * OBJ_UNIT_MM, np.array(faces, dtype=np.int64)


def compact(verts, faces):
    """Drop vertices no face uses and renumber the faces."""
    used = np.unique(faces)
    remap = -np.ones(len(verts), dtype=np.int64)
    remap[used] = np.arange(len(used))
    return verts[used], remap[faces]


def build_links():
    out = bytearray(b"IIWA")
    out += struct.pack("<I", len(LINKS))
    summary = {}
    for name in LINKS:
        verts, faces = read_obj(SRC / "meshes" / f"{name}.obj")
        source_faces = len(faces)
        if name == "A7":
            keep = (verts[faces][:, :, 2] >= A7_MIN_Z_MM).all(axis=1)
            faces = faces[keep]
        verts, faces = compact(verts, faces)
        assert len(verts) < 65536
        out += struct.pack("<II", len(verts), len(faces))
        out += verts.astype("<f4").tobytes()
        out += faces.astype("<u2").tobytes()
        while len(out) % 4:
            out += b"\0"
        summary[name] = {
            "sourceFaces": source_faces,
            "faces": int(len(faces)),
            "vertices": int(len(verts)),
            "min": verts.min(axis=0).round(3).tolist(),
            "max": verts.max(axis=0).round(3).tolist(),
        }
    (OUT / "iiwa7-links.bin").write_bytes(bytes(out))
    return summary


def build_trajectories():
    mat = sio.loadmat(SRC / "KukaDirectDynamics.mat")
    names = sorted((k for k in mat if k.startswith("kukatraj")), key=lambda k: int(k[8:]))
    out = bytearray(b"KDYN")
    out += struct.pack("<III", len(names), SAMPLE_HZ, int(round(1 / TORQUE_STEP_NM)))
    for name in names:
        t = mat[name]
        pos, vel, tau = t[:, 0:7], t[:, 7:14], t[:, 14:21]
        # The velocity columns are exactly the position step times 100 Hz
        # (first row zero), so the file leaves them out and the page derives them.
        derived = np.vstack([np.zeros((1, 7)), np.diff(pos, axis=0) * SAMPLE_HZ])
        assert np.abs(derived - vel).max() == 0, name
        steps = np.round(tau / TORQUE_STEP_NM)
        assert np.abs(steps * TORQUE_STEP_NM - tau).max() < 1e-5, name
        out += struct.pack("<I", len(t))
        out += pos.astype("<f4").tobytes()
        out += steps.astype("<i2").tobytes()
        while len(out) % 4:
            out += b"\0"
    (OUT / "kuka-trajectories.bin").write_bytes(bytes(out))

    # The first 300 rows of run 1 and every 97th row of run 8, at full
    # precision, so the tests can hold the decoders to the source.
    t1, t8 = mat["kukatraj1"], mat["kukatraj8"]
    fixture = {
        "rows": {
            "1": {"index": list(range(300)), "data": t1[:300, :21].tolist()},
            "8": {"index": list(range(0, len(t8), 97)), "data": t8[::97, :21].tolist()},
        },
        "lengths": [int(mat[n].shape[0]) for n in names],
    }
    (FIXTURES / "kuka-rows.json").write_text(json.dumps(fixture))
    return {n: int(mat[n].shape[0]) for n in names}


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    FIXTURES.mkdir(parents=True, exist_ok=True)
    links = build_links()
    (FIXTURES / "links-summary.json").write_text(json.dumps(links, indent=1))
    runs = build_trajectories()
    for name, s in links.items():
        print(f"{name:5} {s['vertices']:5} vertices {s['faces']:5} faces (source {s['sourceFaces']})")
    print("runs", runs)
    for f in sorted(OUT.iterdir()):
        print(f.relative_to(ROOT), f.stat().st_size, "bytes")
