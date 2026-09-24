# Kuka Arm Viz

An interactive WebGL visualization of a Kuka 7-node robot arm, with sliders per joint and inverse kinematics for reaching a target position.

Built in 36-ish hours at RevolutionUC (February 2023). Won the Robotic Data Visualization award (Kinetic Vision) and Best Domain Name (Domain.com/MLH), for `howardgotinastickysituationwithspacearm.tech`.

## What it does

Kinetic Vision brought a challenge to the hackathon: take their Kuka 7-node arm data and make it visible. We built a Unity scene where each of the seven joints has its own slider, and moving one updates the end effector position in real time. The arm also reads the CSV and MATLAB data Kinetic Vision gave us directly, so you can play back their recorded motion instead of only hand-driving it with sliders.

On top of manual control, there are pre-built movement sequences, and inverse kinematics so you can just specify where you want the end effector to go and let the arm solve for the joint angles.

## How it works

The arm is a Unity scene compiled to WebGL. Each of the 7 joint meshes was modeled separately, in Blender, Fusion360, and RoboDK, then exported as `.obj` files (see `BlenderObjects/` and `Kuka_obj_files/`) and rigged so each node's rotation is relative to the node below it. Getting that parent-child rotation chain correct, so rotating a shoulder joint doesn't silently break the wrist below it, was most of the actual engineering.

The pipeline:
- Kinetic Vision's CSV/MATLAB joint data (`Kuka_obj_files/BaxterDirectDynamics.mat`) gets parsed and replayed as a moving sequence.
- Slider input drives per-joint rotation scripts directly, for manual control.
- An inverse kinematics solver takes a target end-effector position and back-solves joint angles.
- The whole thing is built to WebGL and hosted (see `index.html` and the build under `MILLION LINE CSHARP/`).

## Preview the included build

The prebuilt WebGL player is under `MILLION LINE CSHARP/`. From the repository root,
start a local static server and open its index page:

```sh
python3 -m http.server 8000
```

Visit <http://localhost:8000/MILLION%20LINE%20CSHARP/>. The repository contains the
Unity project settings and exported WebGL build, but not the Unity `Assets/` source
tree, so the checked-in build can be previewed but not rebuilt from this checkout.

I worked on the inverse kinematics and movement logic; [Alexander Van Bibber](https://www.linkedin.com/in/alexander-van-bibber/) handled 3D modeling and data processing.

## Prototype

The real thing is Unity and C#. The prototype is neither: `prototype/arm_ik_demo.py` is a synthetic planar 3-link arm in Python that reruns the same two ideas at toy scale — forward kinematics, and inverse kinematics via cyclic coordinate descent (CCD), which rotates one joint at a time toward a target instead of solving a full Jacobian.

Run it locally (Python 3 with Matplotlib and NumPy):

```
python3 -m pip install matplotlib numpy
python3 prototype/arm_ik_demo.py
```

It samples thousands of random joint configurations to trace out the arm's reachable workspace, then solves CCD for one target and plots the resulting pose on top:

![Reachability cloud with one IK solution pose](https://vircgxpcwyvniemqmdyi.supabase.co/storage/v1/object/public/media/writing/Visualizing-Kuka-7-Node-Robot-Arm/reachability_cloud.png)

It also plots each joint's angle across CCD iterations, showing the solver converge to zero end-effector error:

![Joint angle traces converging over CCD iterations](https://vircgxpcwyvniemqmdyi.supabase.co/storage/v1/object/public/media/writing/Visualizing-Kuka-7-Node-Robot-Arm/joint_angle_traces.png)

## Team

- Arya Garg — inverse kinematics and movement logic
- Alexander Van Bibber — 3D modeling and data processing
- [Kaaustaaub Shankar](https://github.com/KaaustaaubShankar)
- [Akhil Penumudy](https://github.com/akhilpenumudy)

Originally hacked together in [KaaustaaubShankar/Visualizing-Kuka-7-Node-Robot-Arm.github.io](https://github.com/KaaustaaubShankar/Visualizing-Kuka-7-Node-Robot-Arm.github.io).

## Links

- [Devpost project page](https://devpost.com/software/visualizing-kuka-7-node-robot-arm)
- [Live demo](https://simmer.io/@aryagarg23/kukavisualisation)
- Writeup: https://aryagarg23.com/writing/kuka-arm-viz
- [aryagarg23.com](https://aryagarg23.com)
- [Devpost profile](https://devpost.com/Aryagarg23)

## More hackathon builds

- [Gyrus](https://github.com/Aryagarg23/Gyrus) — agentic browser that supports curiosity instead of replacing it (WeaveHacks 2025)
- [WhiteBox](https://github.com/Aryagarg23/WhiteBox) — traceable GraphRAG over medical literature (Future of Data 2024, 1st place)
- [G-Code-Assembler](https://github.com/Aryagarg23/G-Code-Assembler) — G-code assembly + STL visualization (MakeUC 2024, Kinetic Vision winner)
- [Terminally-Addicted](https://github.com/Aryagarg23/Terminally-Addicted) — Spotify, GitHub, GPT and YouTube without leaving the terminal (HackOHI/O 2024)
- [Memento](https://github.com/Aryagarg23/Memento) — digital memory journal for Alzheimer's patients and caregivers (RevolutionUC 2024, 3rd overall)
- [Buycott](https://github.com/Aryagarg23/Buycott) — barcode scan -> parent company -> NLP stance on social issues (MakeUC 2023, 1st overall)
- [SignLink](https://github.com/Aryagarg23/SignLink) — video calls with real-time ASL fingerspelling to text (BoilerMake X 2023)
- [Hi-Five](https://github.com/Aryagarg23/Hi-Five) — anonymous friend-matching on OCEAN personality vectors (SASEhack 2024)
- [Friction](https://github.com/Aryagarg23/Friction) — speculative OS + hardware that protects flow state with physical friction (Fig Build 2026)
