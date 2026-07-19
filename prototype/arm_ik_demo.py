"""
Illustrative toy model, NOT the real Kuka rig.

The Unity build drives a 7-DOF arm from CSV/MATLAB joint data with inverse
kinematics for target reaching. This script re-creates the same two ideas
on a synthetic planar 3-link arm: how much space a chain of joints can
reach, and how CCD (cyclic coordinate descent) walks the joints toward a
target frame by frame.
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.style.use("/home/arya/projects/hackathons/.style/garg-paper.mplstyle")

LINK_LENGTHS = np.array([1.0, 0.8, 0.6])
N_LINKS = len(LINK_LENGTHS)
FIG_DIR = os.path.join(os.path.dirname(__file__), "figures")


def forward_kinematics(angles):
    """Joint positions (including base and end effector) for a planar chain."""
    x, y, theta = 0.0, 0.0, 0.0
    points = [(x, y)]
    for length, angle in zip(LINK_LENGTHS, angles):
        theta += angle
        x += length * np.cos(theta)
        y += length * np.sin(theta)
        points.append((x, y))
    return np.array(points)


def ccd_inverse_kinematics(target, iterations=60):
    """Cyclic coordinate descent: rotate one joint at a time toward the target."""
    angles = np.zeros(N_LINKS)
    history = [angles.copy()]
    for _ in range(iterations):
        for i in reversed(range(N_LINKS)):
            points = forward_kinematics(angles)
            joint = points[i]
            end_effector = points[-1]
            to_end = end_effector - joint
            to_target = target - joint
            angle_end = np.arctan2(to_end[1], to_end[0])
            angle_target = np.arctan2(to_target[1], to_target[0])
            delta = angle_target - angle_end
            delta = (delta + np.pi) % (2 * np.pi) - np.pi
            angles[i] += delta
        history.append(angles.copy())
    return angles, np.array(history)


def reachability_cloud(n_samples=6000, rng=None):
    rng = rng or np.random.default_rng(0)
    joint_limit = np.pi
    samples = rng.uniform(-joint_limit, joint_limit, size=(n_samples, N_LINKS))
    ends = np.array([forward_kinematics(a)[-1] for a in samples])
    return ends


def plot_reachability(target, solved_angles):
    ends = reachability_cloud()
    pose = forward_kinematics(solved_angles)
    max_reach = LINK_LENGTHS.sum()

    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    ax.scatter(ends[:, 0], ends[:, 1], alpha=0.3, s=10, color="#3b42db", label="Random joint angles")
    ax.plot(pose[:, 0], pose[:, 1], marker="o", color="#c2491d", linewidth=2.5, label="IK solution pose")
    ax.scatter([target[0]], [target[1]], marker="x", s=80, color="#e85b30", linewidths=2.5, label="Target")
    ax.set_title(f"How much of the plane can a 3-link, {max_reach:.1f} m arm reach?")
    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_aspect("equal")
    ax.legend(loc="upper left")
    fig.savefig(os.path.join(FIG_DIR, "reachability_cloud.png"), dpi=200)
    plt.close(fig)


def plot_joint_traces(history, target, final_error):
    degrees = np.degrees(history)
    fig, ax = plt.subplots(figsize=(6, 4))
    for i in range(N_LINKS):
        ax.plot(degrees[:, i], label=f"Joint {i + 1}")
    ax.set_title(f"How fast does CCD converge on a target? (final error {final_error:.3f} m)")
    ax.set_xlabel("CCD iteration")
    ax.set_ylabel("Joint angle (degrees)")
    ax.legend(loc="best")
    fig.savefig(os.path.join(FIG_DIR, "joint_angle_traces.png"), dpi=200)
    plt.close(fig)


def main():
    os.makedirs(FIG_DIR, exist_ok=True)
    target = np.array([1.6, 0.9])
    solved_angles, history = ccd_inverse_kinematics(target)
    final_pose = forward_kinematics(solved_angles)
    final_error = np.linalg.norm(final_pose[-1] - target)

    plot_reachability(target, solved_angles)
    plot_joint_traces(history, target, final_error)

    print(f"Target: {target}, end effector: {final_pose[-1]}, error: {final_error:.4f} m")


if __name__ == "__main__":
    main()
