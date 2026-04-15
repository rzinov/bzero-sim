TAYCAN PARAMETRIC LAP-TIME MODEL
Akshat Dagdi & Miguel Luque

OVERVIEW
This project is a Simulink-based parametric lap-time and energy model built around the Porsche Taycan Turbo GT Weissach. The aim of the model is to simulate vehicle performance over a Nürburgring Nordschleife drive cycle, then sweep key parameters to understand which variables most strongly influence lap time, regen behaviour, SOC, and net energy use.

The model combines a generated drive cycle, a longitudinal driver, front and rear motor torque application logic, regenerative braking blending, battery behaviour, gearbox logic, and vehicle dynamics. It was developed as an optimisation study rather than a full high-fidelity digital twin, so the emphasis is on directional insight, realistic subsystem interaction, and fast parameter sweeps.

OBJECTIVE
The original target was to benchmark against the official Porsche Taycan Turbo GT Weissach Nürburgring lap record and then explore whether a parameter-optimised setup could improve on the baseline model. The drive-cycle generation stage correlated very closely with the published lap, and the vehicle model was then used as the basis for optimisation and sensitivity work.

MODEL STRUCTURE
The top-level model is split into the following main subsystems:

1. Drive Cycle Input
   - Reads the velocity-versus-distance reference generated from the track map
   - Converts distance progression into reference speed and lap-count tracking
   - Stops the run when the full lap distance is completed

2. Driver
   - Uses reference speed and feedback speed to generate accel/decel commands
   - Provides a simple longitudinal driver layer for tracking the target lap profile

3. Motor
   - Contains front and rear drive torque application
   - Uses motor-speed-based torque limits and applies actual torque demand by scaling available torque with accel demand
   - Separates front and rear outputs to reflect the AWD layout

4. Gearbox
   - Includes the Taycan rear two-speed gearbox behaviour
   - Uses shift logic and clutch logic to determine rear gear ratio application
   - Front axle remains fixed while the rear axle uses the two-speed arrangement

5. Regen + Braking
   - Models blended deceleration strategy
   - Prioritises regenerative braking first, then blends in hydraulic braking
   - Structured to reflect the Taycan strategy of using strong regen before friction braking demand rises

6. Battery
   - Uses a Molicel INR_21700_P45B based R0 battery representation
   - Provides pack voltage, current, and SOC tracking
   - Feeds electrical state outputs into the rest of the model

7. Vehicle / Outputs
   - Converts wheel torques and vehicle forces into longitudinal motion
   - Logs speed, distance, energy, SOC, regen torque, and lap-time outputs
   - Supports comparison between baseline and swept parameter sets

KEY FILES
- DRIVE_CYCLE_GEN.m
  Intermediate drive-cycle generator used to create the velocity-vs-distance profile from the track map. This sits between the earlier point-mass workflow and the FYP drive-cycle generation approach.

- Model_Sweeper.m
  Main batch-run script used for the optimisation study. Updates parameters automatically and runs multiple cases to compare lap time, torque use, SOC, regen energy, and net energy.

- TAYCAN_MODEL.slx
  Main Simulink vehicle model.

- TAYCAN_PARAM_LOAD.m
  Parameter loader for single-case runs.

- VelocityVsDistance.mat
  Saved drive-cycle reference generated from the drive-cycle workflow.

- nordschleife_xy_limits.csv
  Track map / boundary data for the Nordschleife.

- Propulsion in Digital Age Presentation.pdf
  Presentation deck summarising objectives, model developments, optimisation outputs, assumptions, and conclusions.

KEY FEATURES
- Drive-cycle reference generated from track data
- Battery model based on P45B cells using an Rint approach
- Front and rear motor torque application
- Actual torque demand limited by motor torque available at current RPM
- Rear two-speed gearbox logic based on the Taycan architecture
- Blended regen and braking strategy, with regen applied first
- Parametric sweep capability for optimisation studies

OPTIMISATION STUDY
The optimisation study explored how lap time changes with different combinations of:
- power level
- torque scaling
- shift-up / shift-down thresholds
- gearbox strategy
- battery mass / parallel count related settings
- tyre / realism settings
- regen and energy-related behaviour

The study was used to rank configurations by lap time, net energy, regen energy, and SOC outcome, rather than relying on a single nominal setup.

KEY TAKEAWAYS
- The drive-cycle generator was able to reproduce the track benchmark very closely.
- The baseline Simulink model also correlated closely enough to make parameter ranking meaningful.
- Shift strategy, torque scaling, regen strategy, and effective mass had a major influence on results.
- The model is most useful for directional insight and optimisation ranking rather than claiming exact real-world absolute prediction.
- A more realistic driver, thermal effects, tyre refinement, and shift penalties would improve fidelity further.

ASSUMPTIONS / LIMITATIONS
This is not a full validated manufacturer-level vehicle model. Major assumptions include:
- fixed track layout
- fixed vehicle body properties
- fixed powertrain efficiency
- no viscous mechanical loss model
- no steering model
- no environment variation
- isothermal electrical system
- simplified tyre behaviour
- limited mechanical constraints in shifting
- idealised or simplified regen / driver behaviour

FUTURE IMPROVEMENTS
- add battery thermal behaviour
- improve driver realism
- refine tyre and traction modelling
- add environment sensitivity
- include more detailed shift latency and mechanical losses
- improve validation against measured or published vehicle behaviour

OUTPUTS
Typical outputs include:
- lap time
- velocity trace
- speed-vs-distance comparison
- SOC at lap end
- regen energy
- net electrical energy
- average torque use
- parameter ranking between model variants

SUMMARY
This project is a configurable EV lap-time and energy optimisation model built around the Porsche Taycan Turbo GT Weissach. It combines drive-cycle generation, torque-limited acceleration, regen-first braking, a rear two-speed gearbox, and parametric batch studies to explore which variables most strongly influence lap-time performance and energy outcome on the Nordschleife.