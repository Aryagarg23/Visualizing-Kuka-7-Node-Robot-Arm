# BRIEF — Kuka Arm Viz (repo: Aryagarg23/Visualizing-Kuka-7-Node-Robot-Arm)
Slug: kuka-arm-viz. Hackathon: RevolutionUC 2023, February 2023. Prizes: Robotic Data Visualization award (Kinetic Vision), Best Domain Name (Domain.com/MLH) — howardgotinastickysituationwithspacearm.tech.
Team: Arya Garg (inverse kinematics + movement logic), Alexander Van Bibber (3D modeling + data processing), Kaaustaaub Shankar, Akhil Penumudy. Original team repo: https://github.com/KaaustaaubShankar/Visualizing-Kuka-7-Node-Robot-Arm.github.io
What: interactive WebGL visualization of a Kuka 7-node arm — sliders per joint, real-time end-effector position, interprets Kinetic Vision CSV/MATLAB data, pre-built movement sequences, inverse kinematics for target reaching. Unity → WebGL; models via Blender/Fusion360/RoboDK.
Devpost: https://devpost.com/software/visualizing-kuka-7-node-robot-arm
Prototype idea: numpy planar 3-link arm — forward kinematics + simple IK (Jacobian or CCD), chart 1: workspace reachability cloud (scatter, alpha 0.3) with one IK solution pose drawn on top, chart 2: joint angle traces over a reach trajectory (time on x, degrees on y).
