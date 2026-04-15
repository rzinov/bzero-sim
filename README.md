**Overview**

Repository contains custom point-mass 3D physics engine built from scratch in MatLab for the University of Bath Zero electric motorcycle racing team. It was translated into python for RL experimentation in Python. The goal is to simulate vehicle dynamics and optimise lap time, secondary goals include utilising an agent to investigate the potential of a better racing line which takes into accounts the constraints of the physical motorcycle.

**Reinforcement Learning**

To train the agent, the MatLab physics engine which used distance integration was transformed into a python script with a time integration system which allows steps to be completed by that agent.

Agent: Continuous vehicle controller trained to navigate a complex track while managing dynamic grip limits, gear shifts and momentum

Action space: Continuous 2D vector [-1.0, 1.0]

Action[0]: Acceleration/Braking (Scales engine torque based on current gear & RPM)

Action[1]: Target Lean (Change in vehicle lean bounded my maximum lean rates which allows steering)

State space: Normalised 9-dim array:
1. Normalised Vehicle Velocity
2. Normalised Real Lean Angle
3. Cross-track error (dist from centerline)
4. Relative yaw (heading vs track heading)
5. Current gear
6. Engine RPM
7. Lookahead 1: 15 steps ahead
8. Lookahead 2: 35 steps ahead
9. Lookahead 3: 70 steps ahead
    
Reward Function: continuous dense rewards for progression with crash and time penalties:

Positive reward for track progression multiplied by a speed factor.

Base reward for successfully completing a full lap, plus a time bonus scalar.

A penalty and episode termination for going off track, exceeding tyre grip limits (crashes) or moving too slowly.

**Results & Visualisations**

Results are for the initial reinfrocement learning episodes and the MatLab original steady state solver. First 3 images relate to the RL experiment, last 3 relate to the MatLab solver.

<img width="1375" height="752" alt="BATH_RL_track" src="https://github.com/user-attachments/assets/7b9f8cf9-3820-41e5-b55c-21f80017cf36" />
<img width="991" height="593" alt="Bath_RL_post" src="https://github.com/user-attachments/assets/c7fc5abb-696d-4ed2-a4ac-56b82d368338" />
<img width="1041" height="633" alt="Bath_RL_pre" src="https://github.com/user-attachments/assets/50e38102-74bb-46db-a97d-64b4c1347631" />
<img width="1166" height="731" alt="velocity_track" src="https://github.com/user-attachments/assets/8a63d083-cecb-41e8-b5ea-17f34d34a0b3" />
<img width="1145" height="710" alt="VLIM" src="https://github.com/user-attachments/assets/4b5ef4d1-ad2b-4e2d-90e0-23bd0d79721f" />
<img width="1089" height="716" alt="Telem" src="https://github.com/user-attachments/assets/56a3a395-ccee-4c87-aca9-24ef9943b5ef" />

**Repository Structure**

Documentation - Current and previous writeups detailing the MatLab model, how it functions and the supporting academic literature used to create it.

NN - Neural Network experiment, RL models, weights and data.

New Track - CSV file of the track used for these simulations

Sim Gen 1 - MatLab physics engine source code alongside experimentation with different gear systems and direct drive.

Languages: Python, MatLab
Python Env: Jupyter Notebook
