%% Clear workspace and close figures
% Clears all variables, closes all figures, and resets the environment.
close all;
clear all;

%% Data for the EV Vehicle



%% Vehicle Parameters
% In order from model

%% Drive Cycle
load('VelocityVsDistance.mat')
road_grade = 0;

%% Motor
% Specs
Nmotor = 2; % Number of electric motors in the vehicle
Max_torque_total_Nm = 1240; % Maximum torque at launch control (Nm)
System_voltage_V = 800; % (V)
Usable_battery_power_W = 97e3; % (W)

% Motor_Front
Max_torque_motor_F_Nm = 340;  % Maximum torque (Nm) 30/70 Split
Max_power_motor_F_W = 240e3;  % Maximum power (W)
Torque_control_T = 0.2; % Torque control time constant for front, Tc
Motor_eff_F = 95; % Motor Overall Efficiency (η %)
RPM_measure_eff_F_RPM = 9000; % Speed at efficiency measurement (RPM)
Torque_measure_eff_F_Nm = 200; % Torque at efficiency measurement (Nm)
% Motor_Rear
Max_torque_motor_R_Nm = 900;  % Maximum torque (Nm)
Max_power_motor_R_W = 520e3;  % Maximum power (W)
Motor_eff_R = 96; % Motor Overall Efficiency (η %)
RPM_measure_eff_R_RPM = 8000; % Speed at efficiency measurement (RPM)
Torque_measure_eff_R_Nm = 500; % Torque at efficiency measurement (Nm)

%% Acceleration
% Front
Max_torque_curve_F_Nm = [340;340;340;340;340;340;288.55;247.42;216.63;192.44;173.11;157.49;144.19;133.16;123.58]';
RPM_max_RPM_F = 14000;
RPM_curve_RPM_F = (0:1000:RPM_max_RPM_F)'; % RPM curve to 14000 in 1000 increments
% Rear
Max_torque_curve_R_Nm = [900.00;900.00;900.00;900.00;900.00;900.00;900.00;898.90;785.84;698.65;628.92;571.90;524.18;483.85;448.95]';
RPM_max_RPM_R = 14000;
RPM_curve_RPM_R = (0:1000:RPM_max_RPM_R)'; % RPM curve to 14000 in 1000 increments

%% Gearbox
% Front Gearbox
GFront = 8;
% 2Speed Rear Gearbox
GRear_1 = 15;
GRear_2 = 8;
shift_up_v = 80; %180 change if u want harder accel and full gear changes across the model run
shift_down_v = 60; %120 change if u want harder accel and full gear changes across the model run
%% Vehicle
% Diff front
Final_drive_ratio_F = 1/1.4;
% Diff rear
Final_drive_ratio_R = 1/1.4;

%% Tyres
wheel_radius__m = 0.35; % 21" Wheels Taycan Specs Website (m)
Tyre_Crr_pirelli = 0.012; % Rolling resistance endurance slick tyres
% Front
Vertical_load_pirelli_F_N = 6000; % (N)
Longitudinal_load_pirelli_F_N = 71100; % (N)
Peak_tyre_slip_pirelli_F = 10; % peak tyre slip (%)
% Rear
Vertical_load_pirelli_R_N = 6200; % (N)
Longitudinal_load_pirelli_R_N = 71470; % (N)
Peak_tyre_slip_pirelli_R = 10; % peak tyre slip (%)

%% Brakes
Static_Cf = 0.45;
Coulomb_Cf = 0.4;
Break_Fric_v_rads = 0.10;
Viscous_Cf = 0;
% Front
Brake_pad_radius_F_m = 0.168; % Front caliper radius (m)
Cylinder_bore_F_m = 0.044; % Front cylinder bore (m)
% Rear
Brake_pad_radius_R_m = 0.164; % Rear caliper radius (m)
Cylinder_bore_R_m = 0.036; % Rear cylinder bore (m)

%% Regen
regen_bias = 0.3;
Max_brake_pressure_F_Bar = 200e5; %Bar
Max_brake_pressure_R_Bar = 200e5; %Bar
Max_torque_generator_Nm = -350;

%% Battery
% Voltage Scalar
V_init = 4.2;
cell_mass_g = 70;
Imax = 45;
Np = 33;                  % Parallel Count
Ns = 198;                 % Series Count
batt_mass = (Np * Ns * cell_mass_g)/1000;

%% Thermals
T_init = 25; %degC
batt2plate = 2.5e-4;
plate_mass = 20;
plate_heat = 900;
plate2fluid = 5e-4;
inout_thermal = 5026.55; %mm^2
inout_air = 0.25; % m^2
vol = 0.022; 
flow_liquid = 20;
flow_air = 1;

%% Driver
Kpt = 3550; %eff total tractive force (N)
tau = .1; % driver response time (s)
L = 500; %preview distance (m)
aR = 4; % rr coeff (N)
bR = 0.4; % rr coeff (N)
cR = .29; % drag coeff
g = 9.81;

%% Vehicle Body
Axle_wheel_no = 2;
Vehicle_mass_unladen_kg = 2290.0 - 355; % Vehicle mass in kg (Taycan range: 2,290 to 2,710 kg)
Driver_mass_kg = 70; % Driver mass in kg 
Tot_vehicle_mass_kg = Vehicle_mass_unladen_kg + Driver_mass_kg + batt_mass; % Total vehicle mass in kg 
D_CoG_F_m = 1.4; % Distance from CoG to front axle (m)
D_CoG_R_m = 1.8; % Distance from CoG to rear axle (m)
H_CoG_m = 0.35; % Height of centre of gravity (m)
vehicle_frontal_area__m2 = 2.33; % Taycan Specs Sheet (m2)
Cd = 0.29; % Taycan Specs Sheet
rho= 1.225;
