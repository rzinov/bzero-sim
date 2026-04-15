data = readtable('Aragon_Telemetry.csv');
track_dist = data.Distance_m;
target_vel = data.Velocity_ms;
finish_line = track_dist(end);
track_grade = data.Grade_rad;
% 2. BIKE PARAMETERS
bike_mass       = 220;     % [kg] Total mass with rider
frontal_area    = 0.3;     % [m^2]
cd_coeff        = 0.4;     % Drag coefficient
wheel_radius    = 0.3005;  % [m] 0.601/2
final_drive     = 3.68;    % fixed ratio
mech_eff = 0.96; % placeholder b4 dyno
mu_tyre  = 1.4;
Fz_rated = (bike_mass) * 9.81; % [N] Rated vertical load
Fx_peak  = Fz_rated * mu_tyre; % [N] Peak force before slipping
slip_peak = 10; % [%] Slip ratio at peak force
% ^ all guesses
rho = 1.225;
Ad = 0.004;
Bd = 0.000025;
tau = 0.05;
L = 2;
g = 9.81;
aR = bike_mass * g * Ad;
bR = bike_mass * g * Bd;
cR = 0.5 * rho * cd_coeff * frontal_area;
m_wheel_rear = 10; % [kg] rear wheel + tire + sprocket mass
m_wheel_front = 8; % [kg] front wheel + tire + rotor mass
% ^ all guesses
I_wheel_rear = 0.5 * m_wheel_rear * wheel_radius^2;
I_wheel_front = 0.5 * m_wheel_front * wheel_radius^2;
%ASSUMED, NEED CAD
wheelbase = 1.4; % [m] Typical racing wheelbase
D_CoG_F_m = wheelbase * 0.45; % [m] Dist from CG to front axle (slight rear bias)
D_CoG_R_m = wheelbase * 0.55; % [m] Dist from CG to rear axle
H_CoG_m   = 0.5; % [m] Height of CG above ground
% --- Brake system parameters (front wheel only here) ---
rotor_OD                = 0.320;                % [m]
rotor_ID                = 0.246;                % [m]
R_eff                   = 0.5*(rotor_OD + rotor_ID);   % effective pad radius

D_piston                = 33.9e-3;              % [m]
N_piston                = 4;                    % how many pistons
A_piston                = N_piston * pi*(D_piston/2)^2;  % total piston area

mu_pad                  = 0.4;                  % pad friction coeff (guess)
lineP_max               = 1e6;                  % [Pa] ~10 bar, tune to taste

T_brake_max             = 2 * mu_pad * lineP_max * A_piston * R_eff;    % factor 2: two pads
F_brake_wheel_max       = T_brake_max / wheel_radius;                    % [N] at tyre
Viscous_Cf = 0;
% --- OPERATING PRESSURE (The "Gain" values) ---
Max_brake_pressure_F_Bar = 70; % [Bar] Typical racing max
Max_brake_pressure_R_Bar = 20; % [Bar] Lower to allow for Regen
%Guesses
Static_Cf = mu_pad *1.2;
Break_Fric_v_rads = 0.1;
% 3. MOTOR DATA
mot_rpm_vec = [0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000, 7250, 7500];
mot_trq_vec = [120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 114.592, 107.851, 101.859, 96.498, 91.673, 87.308, 83.339, 79.716, 76.394, 73.339, 70.518, 67.906, 65.481, 63.223, 61.115];
eta_max_table           = [0.1, 0.9082, 0.9112, 0.914, 0.9166, 0.9189, 0.921, 0.9229, 0.9246, 0.926, 0.9272, 0.9282, 0.929, 0.9296, 0.9299, 0.93, 0.9299, 0.9296, 0.929, 0.9282, 0.9272, 0.926, 0.9246, 0.9229, 0.921, 0.9189, 0.9166, 0.914, 0.9112, 0.9082, 0.905]; %0.1 to not div by 0
Kpt = (max(mot_trq_vec) * final_drive * mech_eff) / wheel_radius;
power_curve_W = mot_trq_vec .* (mot_rpm_vec * (pi/30));
Max_power_motor_R_W = max(power_curve_W);
Torque_control_T = 0.02; % guess

[max_eff_decimal, peak_index] = max(eta_max_table);
Motor_eff_R = max_eff_decimal * 100; 
RPM_measure_eff_R_RPM = mot_rpm_vec(peak_index); 
Torque_measure_eff_R_Nm = mot_trq_vec(peak_index);

% --- BATTERY PARAMETERS (PLACEHOLDERS) ---
% Based on standard MotoStudent 120V limits using Molicel P45B
Ns = 28;      % Number of cells in series (28s = 117.6V fully charged)
Np = 12;       % Number of cells in parallel
V_init = 117; % Initial starting voltage of the pack
regen_bias = 0.2;
R0_scalar = 0.018; % [Ohm]

SOC_vec = 0:0.01:1;                   % 101 elements
T_vec   = [-40 -30 -20 0 23 45 60];    % 7 elements
R0_P45B_Ohm = R0_scalar * ones(length(SOC_vec), length(T_vec));

v_init = target_vel(1);
wheel_rpm_init = (v_init / wheel_radius) * (60 / (2*pi));