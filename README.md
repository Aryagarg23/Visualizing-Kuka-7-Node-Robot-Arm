# Kuka Arm Viz

A KUKA LBR iiwa 7 R800 in the browser. Play back Kinetic Vision's ten recorded pick-and-place runs with each joint's angle, velocity and torque, turn the joints by hand, or drag a target and let inverse kinematics reach it.

Built in 36-ish hours at RevolutionUC (February 2023). Won the Robotic Data Visualization award (Kinetic Vision) and Best Domain Name (Domain.com/MLH), for `howardgotinastickysituationwithspacearm.tech`. Rebuilt in 2026 as a plain web page.

Live: https://aryagarg23.com/play/kuka-arm

## The challenge

Kinetic Vision asked for a way to see the rotational position, rotational velocity and torque of a 7-joint arm, using a public dataset of a KUKA arm doing ten pick-and-place runs, sampled at 100 Hz ([source/KV_Challenge.pdf](source/KV_Challenge.pdf)).

## What it does

- **Recorded.** Pick a run and play it. The table shows every joint's angle, velocity and torque; each link is shaded by how hard its joint is working, against the largest torque that joint sees in all ten runs. A chart under the table shows all seven torques over the run; click it to jump. The flange's path is drawn in the scene and fills in as the run plays.
- **Joints.** Drag a slider and that joint turns, carrying everything above it.
- **Reach.** Drag the orange target (or use the arrow keys, Page Up and Page Down). The solver finds joint angles that put the flange there with the tool pointing down, starting from the current pose so the arm moves the least it can instead of jumping to a different pose that reaches the same point.

The view is an orthographic isometric drawing: flat surfaces, inked edges and silhouettes, dash-dot joint axes, and drop lines to the floor so heights read without perspective. Drag to turn it, scroll to zoom, double-click to go back to the isometric view.

## How it works

- `source/` holds the inputs from the hackathon: the eight iiwa meshes Kinetic Vision supplied (base and links 1 to 7, in tenths of a millimetre, arm straight up), `KukaDirectDynamics.mat` from the [dataset Kinetic Vision pointed to](https://bitbucket.org/athapoly/datasets/src/master/), and the challenge brief.
- `scripts/build_data.py` turns them into the two files the page loads: `public/data/iiwa7-links.bin` (285 KB) and `public/data/kuka-trajectories.bin` (843 KB). The recorded velocities are exactly the position step times 100 Hz, so the file leaves them out and the page derives them; the script checks that before it drops them. It also drops four stray triangles in the link 7 mesh that sit about 30 cm below the flange.
- `src/kinematics.js` is the arm: joint axes and signs from KUKA's own robot description, forward kinematics, and damped least-squares inverse kinematics on all seven joints (three rows for position, two for tool tilt).
- `src/scene.js` draws it with three.js; `src/main.js` is the page.

The 2023 version was a Unity project compiled to WebGL (a 29 MB download), with the joint chain held together by hand-written rotation scripts. That project, its build and the Blender exports are gone from this branch; they are in the git history.

## Run it

```sh
npm install
npm run dev        # http://localhost:5173
npm test           # data and kinematics tests
npm run build      # static site in dist/
```

`npm run data` rebuilds the data files from `source/` (Python 3 with NumPy and SciPy).

## What the tests hold

- The data files decode to the `.mat` values: positions to 1e-6 rad, derived velocities to 1e-4 rad/s, torques exactly.
- Every joint sits where its two meshes meet.
- With these joint signs, the recordings hold the tool near straight down (median 9° to 17° per run); with joint 4 flipped they would point it up (over 140°). That is how the signs were checked.
- Inverse kinematics reaches 807 flange positions taken from the recordings to within 0.05 mm, inside the joint limits.
- Following run 1's path at 100 Hz, the solver's largest joint step is under three times the robot's own largest step. That is what starting from the current pose buys.

## Still open

- No torque in Joints and Reach modes: the page has no dynamics model, only the recordings.
- Reach only asks for the tool to point down. It does not choose how the tool turns about its own axis, or use the arm's spare motion for anything.
- The dataset's readme calls the arm a "Kuka LWR"; the brief and the meshes are the LBR iiwa 7 R800. The page uses the iiwa's dimensions and limits.
- No collision checks: the arm can pass through itself or the floor.

## Team

- Arya Garg: inverse kinematics and movement logic
- Alexander Van Bibber: 3D modeling and data processing
- [Kaaustaaub Shankar](https://github.com/KaaustaaubShankar)
- [Akhil Penumudy](https://github.com/akhilpenumudy)

Originally hacked together in [KaaustaaubShankar/Visualizing-Kuka-7-Node-Robot-Arm.github.io](https://github.com/KaaustaaubShankar/Visualizing-Kuka-7-Node-Robot-Arm.github.io).

## Links

- [Devpost project page](https://devpost.com/software/visualizing-kuka-7-node-robot-arm)
- [The original Unity build (2023)](https://simmer.io/@aryagarg23/kukavisualisation)
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
