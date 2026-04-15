%% 1. SIM 
% 
% Calculates lap time, energy consumption and top speed for a motorcycle. Also outputs graphs with telemetry and other data
% 
% Inputs:
%   gearRatios = transmission gear ratios [1xN array max 4]
%   isDirectDrive = set to true for direct drive (ignores gears)
%   sprockets = the sprocket configuration for direct drive, default 14,45
%   isDebug = outputs more telemetry data and graphs for debugging

function [lapTime, totalEnergy_J, topSpeed_kmh, Telemetry] = Fly_lap(gearRatios, isDirectDrive, sprockets, isDebug) 
    if nargin < 4
        isDebug = true; % Default to silent
    end
    if nargin < 3
        sprockets = [14, 45]; % Default [Front, Rear] sprocket teeth
    end
    if nargin < 2
        isDirectDrive = true; % Default to direct drive
    end
    if nargin < 1 || isempty(gearRatios)
        gearRatios = [2.5, 1.8, 1.2, 0.77]; 
    end

debugMode = isDebug;

%% 1.1 Basic Cleanup/Init

% Module calling
filename = 'high_fidel_track.csv';
[trackDataOut] = processTrack(filename);

% Import vars from processTrack module
xresMCP_laps        = trackDataOut.xresMCP_laps;
yresMCP_laps        = trackDataOut.yresMCP_laps;
segmentLengths      = trackDataOut.segmentLengths;
RProfile            = trackDataOut.RProfile;
TSignProfile        = trackDataOut.TSignProfile;
trajMCP             = [xresMCP_laps, yresMCP_laps];
zt                  = trackDataOut.zt;
finalStepLocs       = trackDataOut.finalStepLocs;
scale_factor        = trackDataOut.scale_factor;
xin                 = trackDataOut.xin;
xout                = trackDataOut.xout;
yin                 = trackDataOut.yin;
yout                = trackDataOut.yout;
name                = trackDataOut.name;
xt                  = trackDataOut.xt;
yt                  = trackDataOut.yt;
xresMCP             = trackDataOut.xresMCP;
yresMCP             = trackDataOut.yresMCP;
bankingNoise      = trackDataOut.banking;

% Link start/finish
% Forces the start and end to be infinite straights to prevent sharp turns/decel.
RProfile(1:20) = inf; 
RProfile(end-20:end) = inf; 
RProfile(RProfile < 0.1) = inf; 

raw_curvature = 1 ./ RProfile;
raw_curvature(isnan(raw_curvature)) = 0;

% Smooth the curvature to remove spikes and make simulation stable (actual driver uses real unsmooth track)
% Done to mimic the smooth steering inputs of a real driver and provide a best case scenario
clean_curvature = smoothdata(raw_curvature, 'gaussian', 10);

% Convert back to radius for VLIM
RProfile_Clean = 1 ./ clean_curvature;

% Clamps Infs and tiny radii
max_straight_R = 100000000;
RProfile_Clean(abs(RProfile_Clean) > max_straight_R) = max_straight_R;

s_track = zeros(length(zt), 1);
for idx = 1:length(segmentLengths)
    s_track(idx+1) = s_track(idx) + segmentLengths(idx);
end

% Smoothing banking profile
distance_vec = s_track(1:end-1); 
avg_dx = mean(diff(distance_vec));
window_distance_m = 45; 
window_points = round(window_distance_m / avg_dx); 
bankingProfile = smoothdata(bankingNoise, 'gaussian', window_points);

% 1st derivative (slope) and 2nd derivative (rate of slope change)
dz_ds = gradient(zt(:), s_track(:));
d2z_ds2 = gradient(dz_ds(:), s_track(:));

% Vertical curvature K (+ dip/compression, - crest/unweighting)
K_vert = d2z_ds2 ./ (1 + dz_ds.^2).^(1.5);

% =========================================================================

% 1.2 Specific Constants

% --- Vehicle Specs ---
wheelRadius             = 0.601/2; % meters
frontalArea             = 0.3; % square meters
P_aux                   = 150; % Auxiliary power draw (ECU, dash, water pump, cooling fans) - guesstimate, adjust as needed
carMass                 = 220; % kg
Me_scalingfactor        = 1.1;
M_effective             = carMass * Me_scalingfactor;
speed_limit             = 85;
maxV                    = 85;

% --- Aero & Env ---
cd                      = 0.4; % Drag coefficient
rho                     = 1.225; % kg/m^3
h_cog                   = 0.45; %Cog bike
maxLeanDeg              = 55;
maxLeanRad              = deg2rad(maxLeanDeg);
maxLeanRateDeg          = 30;           % e.g. 60 deg/s, tune as you like
maxLeanRate             = deg2rad(maxLeanRateDeg);  % [rad/s]
g                       = 9.81; % Gravitational acceleration in m/s²
Ad                      = 0.004; % Rolling resistance coefficient (velocity-independent)
Bd                      = 0.000025; % Rolling resistance coefficient (velocity-dependent)
useLeanRateClamp        = true;
useMaxLeanClamp         = true;

% --- Powertrain ---
finalDriveRatio         = 3.68;
chainDriveRatio         = sprockets(2) / sprockets(1); % Used if direct drive is active

if isDirectDrive || isscalar(gearRatios)
    mech_eff            = 0.96;  % Direct drive efficiency, adjust as needed
    isDirectDrive = true;    % Enforce direct drive if 1 gear is passed
else
    mech_eff            = 0.93;  % 2-speed or 3-speed efficiency, guesstimate again, adjust as needed.
end

shift_delay_time        = 0.050;    % 50ms shift delay where force = 0, adjust as needed
shift_timer             = 0;             
current_gear            = 1;            % Init gear is 1

shift_cooldown_time     = 1.0;   % Minimum 1 second between shifts
shift_cooldown_timer    = 0;

% Specific motor data
rpm_table               = [0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000, 7250, 7500];
T_max_table             = [120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 114.592, 107.851, 101.859, 96.498, 91.673, 87.308, 83.339, 79.716, 76.394, 73.339, 70.518, 67.906, 65.481, 63.223, 61.115];
eta_max_table           = [0, 0.9082, 0.9112, 0.914, 0.9166, 0.9189, 0.921, 0.9229, 0.9246, 0.926, 0.9272, 0.9282, 0.929, 0.9296, 0.9299, 0.93, 0.9299, 0.9296, 0.929, 0.9282, 0.9272, 0.926, 0.9246, 0.9229, 0.921, 0.9189, 0.9166, 0.914, 0.9112, 0.9082, 0.905];
motor_redline           = 7500;

% --- Tyre specs ---

t_tyre                  = 67.8/1000; % tyre thickness bike
tireFrictionCoeff       = 1.4; % Maximum friction coefficient mu

% Bridgestone estimate from spec sheets, contact area from Force/Pressure given weight of 220kg, 
% Official data estimates 200,000pa pressure for MotoStudent tyres. Contact area increase of 20% up to 55 degrees
leanDeg_AreaTable       = [0 5 10 15 20 25 30 35 40 45 50 55];
contactAreaTable        = [0.0054 0.0054 0.0054 0.0055 0.0055 0.0056...
                            0.0057 0.0058 0.0059 0.0061 0.0063 0.0065];

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
F_brake_wheel_max       = T_brake_max / wheelRadius;                    % [N] at tyre

% ===========================================================================

% 1.3 Init starting conditions
numLaps = 2;
velocity = 0.1;     % Init velocity in m/s
time = 0;           % Init time
totalEnergy_J = 0;  % Init energy

% Preallocation (faster outside of loop)
numSteps = length(xresMCP_laps) - 1;

% Pre-allocate all arrays with zeros
FCMDprofile     = zeros(numSteps, 1);
DTprofile       = zeros(numSteps, 1);
velocityProfile = zeros(numSteps, 1);
timeProfile     = zeros(numSteps, 1);
accel           = zeros(numSteps, 1);
F_power_array   = zeros(numSteps, 1);
F_remain_array  = zeros(numSteps, 1);
F_applied       = zeros(numSteps, 1);
FTyreprofile    = zeros(numSteps, 1);
Aprofile        = zeros(numSteps, 1);
Muprofile       = zeros(numSteps, 1);
powerProfile    = zeros(numSteps, 1); 
leanProfile     = zeros(length(xresMCP_laps)-1, 1);

% ===========================================================================

%% 2. Best possible velocity profile VLIM
% Initialises variables and precalcs

leanRad_AreaTable = deg2rad(leanDeg_AreaTable); % convert degrees to rad

A0 = contactAreaTable(1);                       % 1st value in table, reference contact area
mu_available_curve = tireFrictionCoeff * (contactAreaTable / A0); % scale friction coefficient base on contact patch

mu_required_curve = tan(leanRad_AreaTable);
diff_curve = mu_available_curve - mu_required_curve; %grip margin (positive = safe, negative = sliding)

interp_angles = linspace(0, deg2rad(55), 1000);
diff_interp = interp1(leanRad_AreaTable, diff_curve, interp_angles, 'pchip'); % creates a smooth interpolation to find exact limit more precisely

[~, idx_cross] = min(abs(diff_interp)); % identify the lean angle where available grip equals required grip (diff closest to 0)
limit_lean_angle = interp_angles(idx_cross);

mu_at_limit = interp1(leanRad_AreaTable, mu_available_curve, limit_lean_angle, 'pchip');
mu_corn_dynamic = mu_at_limit * 0.98;       % 2% safety margin so its not razor thin

VLIMprofile = zeros(length(xresMCP_laps)-1, 1); % init VLIMprofile

a_mech_limit = F_brake_wheel_max / M_effective; % Max braking decel
safety_factor = 1.0; % Safety factor to adjust vlim, was used before but not needed now
phi_prev    = 0;

% ===========================================================================

% 2.1 Geometric limit (cornering speed)
for lap = 1:numLaps
    % Reset lap-specific accumulators for each pass
    time = 0;
    totalEnergy_J = 0;
    phi_prev = 0;
for k = 1:length(xresMCP_laps)-1
    R_k = abs(RProfile_Clean(k)); 

    % Only process corners (ignore straights or infinite radii)
    if isfinite(R_k) && R_k > 0 && R_k < 10000

        % Camber direction
        turnSign = TSignProfile(k);
        theta_bank = bankingProfile(k);
        
        % If Turn and Camber have same sign -> Good (+), If different signs -> Bad (-)
        if sign(theta_bank) == sign(turnSign)
             theta_bank_effective = abs(theta_bank);
        else
             theta_bank_effective = -abs(theta_bank);
        end

        % Tire friction limit (sliding), using the bank turn formula: v = sqrt(g * R * (mu + tan(theta)) / (1 - mu*tan(theta)))
        num = mu_corn_dynamic + tan(theta_bank_effective);
        den = 1 - mu_corn_dynamic * tan(theta_bank_effective);

        % Prevent division by zero or negative values in high banked turns
        if den < 0.01, den = 0.01; end
        v_friction = sqrt(g * R_k * (num / den));
        
        % Hard lean limit (geometric), constrain lean angle by physical bike limits or tyre profile limits
        real_max_lean_rad = min(maxLeanRad, limit_lean_angle);

        % Calculate total effective lean relative to gravity (Bike lean + track banking)
        effective_max_lean = real_max_lean_rad + theta_bank_effective;

        % Prevents 90-degree lean which results in infinite speed
        if effective_max_lean >= pi/2, effective_max_lean = pi/2 - 0.01; end
        v_lean_geometry = sqrt(g * R_k * tan(effective_max_lean));
        

        % Final velocity constraint, speed limited by most restrictive factor (friction, geometry, top speed)
        VLIMprofile(k) = min([v_friction, v_lean_geometry, speed_limit]);
    else
        % Default to top speed on straights
        VLIMprofile(k) = speed_limit;
    end
end

% ===========================================================================

% 2.2 Backwards Pass

for pass = 1:3
    for k = length(VLIMprofile)-1:-1:1
        
        % --- GEOMETRY & PRELIM ---
        dist = segmentLengths(k);
        v_next = VLIMprofile(k+1);
        R_here = abs(RProfile_Clean(k));
        turnSign = TSignProfile(k);
        theta_bank = bankingProfile(k);
        
        % Ensure banking sign assists or opposes the turn correctly
        theta_bank_effective = sign(turnSign) * theta_bank;
        
        % Estimate mid-segment velocity for more accurate (aero) force calculations
        v_est_mid = sqrt(v_next^2 + a_mech_limit * dist); 
        
        % --- LEAN & TYRE CONTACT ---

        % Calculate lean angle relative to the road surface
        if isfinite(R_here) && R_here > 0 && R_here < 10000
            phi_flat = atan(v_est_mid^2 / (g * R_here));
            phi_ideal = phi_flat - theta_bank_effective;
     
        % Account for tyre thickness
            if (h_cog > t_tyre) && (phi_ideal ~= 0)
                phi_delta = asin((t_tyre * sin(phi_ideal)) / (h_cog - t_tyre));
                phi = phi_ideal + phi_delta;
            else
                phi = phi_ideal;
            end
        else
            phi = 0;
        end

        % --- FRICTION PENALTY ---
        phi_relative_to_road = rad2deg(abs(phi));
        A_contact = interp1(leanDeg_AreaTable, contactAreaTable, phi_relative_to_road, 'linear', 'extrap');
        Aprofile(k) = A_contact;

        mu_eff = tireFrictionCoeff * (A_contact / A0)^0.15; %0.15 is a guess value

        % --- ELEVATION & DYNAMIC NORMAL FORCE  ---
        % Calculate pitch/slope for this specific segment early
        dz = zt(k+1) - zt(k);
        sin_theta = dz / sqrt(dist^2 + dz^2);
        cos_theta = sqrt(1 - sin_theta^2);
        
        K_v = K_vert(k); % Local vertical curvature
        Fz_vert = carMass * (v_est_mid^2) * K_v;
        
        Fz_bp = (carMass * g * cos(theta_bank_effective) * cos_theta) + ...
                (carMass * (v_est_mid^2 / max(1, R_here)) * sin(theta_bank_effective)) + Fz_vert;
                
        Fz_bp = max(0.1, Fz_bp); % Safety clamp to prevent negative normal force (flying)

        % --- TRACTION CIRCLE ---
        % Max deceleration the TIRE can generate (Force/M_eff)
        f_friction = mu_eff * Fz_bp; 
        a_grip_max = f_friction / M_effective;

        v_est = sqrt(v_next^2 + 2 * a_mech_limit * dist);
        v_mean_bp = (v_next + v_est) / 2;

        % Lateral acceleration required (geometric)

        a_lat_geometric = v_mean_bp^2 / max(1, R_here);
        a_banking_assist = g * tan(theta_bank_effective);
        a_lat_demand_tire = abs(a_lat_geometric - a_banking_assist);

        f_lat_demand = carMass * a_lat_demand_tire;
        a_lat_scaled = f_lat_demand / M_effective;
    
        % if lateral force required is greater than can provide, force speed down
        if a_lat_scaled >= a_grip_max
            % corner is too sharp for current speed; solve for max physical cornering speed
            num = mu_eff + tan(theta_bank_effective);
            den = 1 - mu_eff * tan(theta_bank_effective);
            den = max(0.01, den);

            v_physics_limit = sqrt(g * R_here * (num / den));
            VLIMprofile(k) = min(VLIMprofile(k), v_physics_limit);
            
            % Reset local integratin params
            v_next = VLIMprofile(k); 
            a_grip_available = 0;       
        else
             a_grip_available = sqrt(a_grip_max^2 - a_lat_scaled^2); %longit grip available
        end
        
        % Limit decel by mechanical braking system capabil
        a_brake_limit_local = min(a_mech_limit, a_grip_available);

        % --- LONGIT RESISTANCE & INTEGRATION ---
        a_aero = (0.5 * rho * cd * frontalArea * v_next^2)/ M_effective;
        a_roll = (carMass * 9.81 * (Ad + Bd * v_next))/M_effective;
        a_grav = (carMass * 9.81 * sin_theta)/M_effective; 

        % Total decel
        a_total_decel = (a_brake_limit_local * safety_factor) + a_aero + a_roll + a_grav;
        a_total_decel = max(0.01, a_total_decel); % only a braking force
        
        % Kinematic integration 
        v_brake_limit = sqrt(v_next^2 + 2 * a_total_decel * dist);
        VLIMprofile(k) = min(VLIMprofile(k), v_brake_limit);
    end
    
    % Closes loop for velocity profile
    v_start = VLIMprofile(1);
    v_end_max = sqrt(v_start^2 + 2 * (a_mech_limit * safety_factor) * segmentLengths(end));
    VLIMprofile(end) = min(VLIMprofile(end), v_end_max);
end

% ==============================================================================================

%% 3. Simulation loop with power-sensitive lap time scaling
for i = 1:length(xresMCP_laps)-1
    
    % 3.1 Local Track Geometry

    turnSign = TSignProfile(i);
    r_turn = RProfile(i);
    theta_bank = bankingProfile(i);
    ds_seg = segmentLengths(i);

    % Elevation
    dz = zt(i+1) - zt(i);
    sin_theta = dz / sqrt(ds_seg^2 + dz^2);
    cos_theta = sqrt(1 - sin_theta^2);

% ==============================================================================================

    % 3.2 Target lean angle

    theta_bank_effective = sign(turnSign) * theta_bank;

    % --- Roll angle from Cossalter 4.1.1 & 4.1.2 ---
    if isfinite(r_turn) && r_turn > 0
        % Calculate lean needed on flat surface
        phi_flat = atan(velocity^2/(g*r_turn));
        % Adjust for banking (banking reduces required lean)
        phi_i = phi_flat - theta_bank_effective;
    else
        phi_i = 0;
    end

    % extra roll due to tyre thickness
    if (h_cog > t_tyre) && (phi_i ~= 0)
        phi_delta = asin((t_tyre * sin(phi_i)) / (h_cog - t_tyre));
    else
        phi_delta = 0;
    end

    phi_mag = phi_i + phi_delta;   % magnitude of effective roll, always ≥0
    phi_signed_target = -sign(turnSign) * phi_mag; % Left is negative

% ==============================================================================================

    % 3.3 Lean Dynamics

    phi_target = phi_signed_target;   % from previous section

    % Physical lean clamp
    if useMaxLeanClamp
        phi_target = max(min(phi_target, +maxLeanRad), -maxLeanRad);
    end

    % Lean rate tapering (smoothness factor)
    % Taper the roll rate to 0 as max lean is approached to prevent snapping or overshoot
    taper_start_angle = 0.7 * maxLeanRad;   % tapering starts at 70%
    if abs(phi_prev) > taper_start_angle
        ratio_in_zone = (abs(phi_prev) - taper_start_angle) / (maxLeanRad - taper_start_angle);
        taper = 1 - min(ratio_in_zone, 1)^2; 
    else
        taper = 1;
    end         
    
    maxLeanRate_eff = maxLeanRate * taper;

    % Limits how fast the bike can transition from left to right
    if useLeanRateClamp
        maxDeltaPhi = maxLeanRate_eff * ds_seg / max(velocity, 1);
        dphi_req = phi_target - phi_prev;

        if abs(dphi_req) > maxDeltaPhi
            phi_actual = phi_prev + maxDeltaPhi * sign(dphi_req);
        else
            phi_actual = phi_target;
        end
    else
        phi_actual         = phi_target;
    end

    leanProfile(i) = phi_actual;
    phi_prev       = phi_actual;
    phi_road_rel = abs(phi_actual);


% ==============================================================================================

    % 3.4 Tyre Friction

    % Map lean angle to contact area and adjust Mu accordingly
    A_contact    = interp1(leanDeg_AreaTable, contactAreaTable, rad2deg(phi_road_rel), 'linear', 'extrap');
    Aprofile(i) = A_contact;

    mu_adjust    = (A_contact / A0)^0.15; % 0.15 is estimate  
    mu_eff       = tireFrictionCoeff * mu_adjust;  
    Muprofile(i) = mu_eff;

% ==============================================================================================

    % 3.5 Forces & Traction Lim

    % Lateral force
    if r_turn ~= Inf 
        F_centrifugal_out = carMass * velocity^2 / r_turn * cos(theta_bank_effective);
        F_gravity_in = carMass * g * sin(theta_bank_effective);
        F_lat_demand = abs(F_centrifugal_out - F_gravity_in);
    else 
        F_lat_demand = 0; 
    end

    % Normal Force
    K_v = K_vert(i);
    Fz_vert = carMass * (velocity^2) * K_v;
    
    Fz = (carMass * g * cos(theta_bank_effective) * cos_theta) + ...
         (carMass * (velocity^2 / r_turn) * sin(theta_bank_effective)) + ...
         Fz_vert;
    
    % Ensure Fz is never negative (prevents imaginary numbers in sqrt if bike jumps)
    Fz = max(0.1, Fz);

    % --- TRACTION LIMIT ---

    % Total available friction capacity
    F_tire_total = mu_eff * Fz; 
    FTyreprofile(i) = F_tire_total;

    % Remaining grip available for accel/braking, calculates how much required to hold turn (F_lat).
    if F_lat_demand > F_tire_total
        F_long_cap = 0;     % The tire cannot hold the turn even with 0 braking/gas. 
    else
        F_long_cap = sqrt(F_tire_total^2 - F_lat_demand^2);
    end

% ==============================================================================================

    % 3.6 Powertrain & Auto-Shifter Logic

    whl_rpm = (velocity * 30) / (pi * wheelRadius);
    
    if isDirectDrive
        % --- A. DIRECT DRIVE CHAIN ---

        mot_rpm = whl_rpm * chainDriveRatio; 
        
        % Smooth overrev taper instead of instant 0Nm wall
        if mot_rpm <= motor_redline
            T_avail = interp1(rpm_table, T_max_table, mot_rpm, 'linear', 0);
        else
            rpm_over = mot_rpm - motor_redline;
            taper_range = 500; 
            T_avail_at_redline = T_max_table(end);
            T_avail = max(0, T_avail_at_redline * (1 - (rpm_over / taper_range)));
        end
        
        % Convert motor torque to linear wheel force
        F_power_limit = (T_avail * chainDriveRatio * mech_eff) / wheelRadius;
        active_eta = interp1(rpm_table, eta_max_table, mot_rpm, 'linear', 0.85);
        
    else
        % --- B. MULTI-GEAR AUTO-SHIFTER ---

        % Decay shift timers to not spam shifting (safety lock)
        if i > 1
            if shift_timer > 0
                shift_timer = max(0, shift_timer - DTprofile(i-1)); 
            end
            if shift_cooldown_timer > 0
                shift_cooldown_timer = max(0, shift_cooldown_timer - DTprofile(i-1));
            end
        end
        
        % Calculate the RPM in current gear
        mot_rpm_current = whl_rpm * gearRatios(current_gear) * finalDriveRatio;
        best_gear = current_gear;
        
        % Shift Logic (triggers based on RPM)
        if shift_cooldown_timer <= 0
            % UPSHIFT: If we are revving past 7400 RPM (As 7500 is redline)
            if current_gear < length(gearRatios) && mot_rpm_current > 7400
                best_gear = current_gear + 1;
                
            % DOWNSHIFT: If shifting down puts the engine at 7100 RPM or lower
            elseif current_gear > 1
                mot_rpm_down = whl_rpm * gearRatios(current_gear - 1) * finalDriveRatio;
                if mot_rpm_down < 7100 
                    best_gear = current_gear - 1;
                end
            end
        end
        
        % Apply the time penalty after shifting
        if best_gear ~= current_gear
            shift_timer = shift_delay_time;        
            shift_cooldown_timer = shift_cooldown_time; 
            current_gear = best_gear;
        end
        
        % Calculate tractive force for gear
        mot_rpm_active = whl_rpm * gearRatios(current_gear) * finalDriveRatio;
        if mot_rpm_active <= motor_redline
            T_avail = interp1(rpm_table, T_max_table, mot_rpm_active, 'linear', 0);
        else
            rpm_over = mot_rpm_active - motor_redline;
            T_avail = max(0, T_max_table(end) * (1 - (rpm_over / 500)));
        end
        
        max_tractive_force = (T_avail * gearRatios(current_gear) * finalDriveRatio * mech_eff) / wheelRadius;
        active_eta = interp1(rpm_table, eta_max_table, mot_rpm_active, 'linear', 0.85);
        
        % Cut power during shift
        if shift_timer > 0
            F_power_limit = 0; 
        else
            F_power_limit = max_tractive_force;
        end
    end

% ==============================================================================================

    % 3.7 Driver decision logic

    % Decides whether to gas or brake (bang bang controller)
    % looks at how far above / below the local speed limit we are

    % Target speed for the end of this specific segment
    if i < length(VLIMprofile)
        v_target_next = VLIMprofile(i+1); 
    else
        % On last segment of lap hold the final speed limit
        v_target_next = VLIMprofile(i); 
    end    

    % Calculate the exact acceleration needed to hit that target speed
    a_req = (v_target_next^2 - velocity^2) / (2 * segmentLengths(i));
    
    % Estimate resisting forces for the current step
    F_drag_est = 0.5 * cd * rho * frontalArea * velocity^2;
    F_roll_est = carMass * g * (Ad + Bd * velocity);
    F_grav_est = carMass * g * sin_theta;
    
    % Calculate the exact longitudinal force required at the contact patch
    F_cmd = (a_req * M_effective) + F_drag_est + F_roll_est + F_grav_est;
    
    % --- APPLYING PHYSICAL LIMS ---
    % Clamps the ideal force to reality

    if F_cmd >= 0
        % Accelerating
        F_long = min([F_cmd, F_long_cap, F_power_limit]);
    else
        % Braking (Clamped to prevent asking for impossible grip)
        F_long = max([F_cmd, -F_brake_wheel_max]);    % F_long_cap not used because integration step is too large to not cause instability and crash the model
    end
    
    FCMDprofile(i) = F_cmd;

% ==============================================================================================


    % 3.8 Applying final forces

    % Init
    v_start = velocity;
    ds = segmentLengths(i);

    F_drag = 0.5 * cd * rho * frontalArea * velocity^2;
    F_roll = carMass * g * (Ad + Bd * velocity);
    F_grav = carMass * g * sin_theta;
    
    V66 = min(speed_limit, maxV) * 0.95;
    F_long_taper = F_long;

    %  Only taper during acceleration
    if (v_start > V66) && (F_long > 0)
        ratio        = (v_start - V66) / (min(speed_limit, maxV) - V66);
        taper        = max(0, 1 - ratio);
        F_long_taper = F_long * taper;
    end
    
    % Predictor-Corrector Integration (Heun's)
    % Since drag scales with v^2, calculating drag only at the start of the segment causes errors. "Predict" end speed to avg the drag

    % A: Predictor (Init estimate)
    F_net_start = F_long_taper - F_drag - F_roll - F_grav;
    a_start = F_net_start / M_effective;

    v_end_est = sqrt(max(0.1, v_start^2 + 2 * a_start * ds));
    F_drag_end = 0.5 * cd * rho * frontalArea * v_end_est^2;

    % B: Corrector
    F_net_avg = F_long_taper - (F_drag + F_drag_end)/2 - F_roll - F_grav; %avg drag as drag depends on velocity
    dv_avg = F_net_avg / M_effective;

    % Final velocity for this step
    v_sq = v_start^2 + 2 * dv_avg * ds;
    velocity = sqrt(max(0.1, v_sq));

    % Time step based on average velocity
    v_mean = (v_start + velocity) / 2;
    dt_seg = ds / max(0.1, v_mean);

% ==============================================================================================

    % 3.9 Storing & Updating
    velocityProfile(i)  = velocity;
    timeProfile(i)      = time;
    accel(i)            = dv_avg;
    DTprofile(i)        = dt_seg;
    F_power_array(i)    = F_power_limit;
    F_remain_array(i)   = F_long_cap;
    F_applied(i)        = F_long;

    % Total energy used
    p_mech_wheel = F_long_taper * v_mean; 
    
    if p_mech_wheel > 0
        % Mechanical power / efficiency + Auxiliary systems
        instPower = (p_mech_wheel / active_eta) + P_aux;
    else
        % Coasting/Braking: Only auxiliary draw
        instPower = P_aux; 
    end
    
    powerProfile(i) = instPower;
    totalEnergy_J   = totalEnergy_J + (instPower * dt_seg);

    %Lap Time
    time = time + dt_seg;
end % --- END OF SIMULATION LOOP ---
    if lap == 1
        fprintf('Lap 1 (Warm-up) complete. Entry speed for Flying Lap: %.2f km/h\n', velocity * 3.6);
    end
end
% ==============================================================================================

%% 4. Assign the variables to the workspace

% Export key arrays into a struct
Telemetry.timeProfile     = timeProfile;
Telemetry.velocityProfile = velocityProfile;
Telemetry.VLIMProfile     = VLIMprofile;
Telemetry.accel           = accel;
Telemetry.Fcom            = [timeProfile, FCMDprofile];
Telemetry.mu              = [timeProfile, Muprofile];
Telemetry.DC              = [timeProfile, velocityProfile];
Telemetry.leanProfile     = leanProfile;
Telemetry.distance        = s_track(1:end-1);
Telemetry.F_applied       = F_applied;

% Metrics
lapTime      = timeProfile(end);
total_track_length = s_track(end);
avg_speed_ms = total_track_length / lapTime;
topSpeed_kmh = max(velocityProfile) * 3.6; 
    
% Print Results to Command Window
fprintf('Lap time:              %.3f s\n', lapTime);
fprintf('Average vehicle speed: %.2f m/s (%.2f km/h)\n', avg_speed_ms, avg_speed_ms * 3.6);
fprintf('Top speed achieved:    %.2f km/h\n', topSpeed_kmh);

% ==============================================================================================

%% 5. Plots

if debugMode
        
        % Energy metrics
        totalEnergy_Wh  = totalEnergy_J / 3600;
        totalEnergy_kWh = totalEnergy_Wh / 1000;
        track_length_km = trackDataOut.segmentLengths(end) * numSteps / 1000;
        
        fprintf('Energy Consumed:       %.2f Wh (%.4f kWh)\n', totalEnergy_Wh, totalEnergy_kWh);
        fprintf('Energy Economy:        %.2f Wh/km\n', totalEnergy_Wh / track_length_km);
        
        % 5.1 Track geometry plots
        
        % 3D Elevation Plot
        figure('Name', '3D Elevation');
        X_surf = [xin(:)'; xout(:)'];
        Y_surf = [yin(:)'; yout(:)'];
        Z_surf = [zt(:)'; zt(:)']; 
        s = surf(X_surf, Y_surf, Z_surf, Z_surf);
        s.EdgeColor = 'none';    
        s.FaceColor = 'interp'; 
        colorbar;              
        title('3D Track Elevation');
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        axis equal; view(3); 
        
        % Track Boundaries & Minimum Curvature Path
        figure('Name', 'Boundaries & Racing Line');
        hold on;
        plot([xin(1) xout(1)], [yin(1) yout(1)], 'k', 'LineWidth', 2, 'DisplayName', 'Starting Line');
        plot(xt, yt, 'k--', 'LineWidth', 1, 'DisplayName', 'Reference Line');
        plot(xin, yin, 'r', 'LineWidth', 1, 'DisplayName', 'Inner Track');
        plot(xout, yout, 'r', 'LineWidth', 1, 'DisplayName', 'Outer Track');
        plot(xresMCP, yresMCP, 'b', 'LineWidth', 2, 'DisplayName', 'Racing Line');
        hold off;
        xlabel('X (m)', 'FontWeight', 'bold'); ylabel('Y (m)', 'FontWeight', 'bold');
        title('Minimum Curvature Trajectory', 'FontWeight', 'bold');
        legend('Location', 'best'); grid on; axis equal;

        % Track Elevation (2D)
        figure('Name', 'Track Geometry: 2D Elevation vs Distance');
        plot(finalStepLocs * scale_factor, zt, 'LineWidth', 2);
        xlabel('Distance (m)', 'FontWeight', 'bold'); ylabel('Elevation (m)', 'FontWeight', 'bold');
        title('Track Elevation Profile', 'FontWeight', 'bold');
        grid on;
        
% ==============================================================================================

        % 5.2 Vehicle dynamics plots
        
        % Velocity Gradient on Track Layout
        figure('Name', 'Velocity Map');
        scatter(xresMCP_laps(1:end-1), yresMCP_laps(1:end-1), 20, velocityProfile, 'filled');
        colormap(turbo); c = colorbar;
        c.Label.String = 'Velocity (m/s)'; c.Label.FontWeight = 'bold';
        clim([0 max(velocityProfile)]); 
        xlabel('X (m)', 'FontWeight', 'bold'); ylabel('Y (m)', 'FontWeight', 'bold');
        title('Velocity Gradient Over Track', 'FontWeight', 'bold');
        grid on; axis equal;

        % Velocity vs Speed Limit
        figure('Name', 'Velocity vs VLIM');
        hold on;
        plot(timeProfile, velocityProfile, 'r', 'LineWidth', 1.5, 'DisplayName', 'Actual Speed');
        plot(timeProfile, VLIMprofile, 'g', 'LineWidth', 1.5, 'DisplayName', 'Speed Limit (Grip/Braking)');
        xlabel('Time (s)'); ylabel('Speed (m/s)');
        title('Vehicle Speed vs Physical Limit');
        legend('Location', 'best'); grid on; hold off;
        
        % Force Limiting Conditions
        figure('Name', 'Force Application vs Limits');
        hold on;
        plot(F_applied, 'k', 'LineWidth', 1.5, 'DisplayName', 'Applied Force'); 
        plot(F_power_array, 'r--', 'DisplayName', 'Powertrain Limit');
        plot(F_remain_array, 'g--', 'DisplayName', 'Tire Grip Limit (Longitudinal)');
        xlabel('Segment Index'); ylabel('Force (N)');
        title('Longitudinal Force Limiting Conditions');
        ylim([-3000 6000]); legend('Location', 'best'); grid on; hold off;

        % Lean Angle Over Lap
        figure('Name', 'Dynamics: Lean Angle');
        plot(timeProfile, rad2deg(leanProfile), 'LineWidth', 1.5);
        xlabel('Time (s)'); ylabel('Lean Angle (deg)');
        title('Lean Angle Over Lap'); grid on;

        % Contact Area & Friction
        figure('Name', 'Tire Grip & Area');
        yyaxis left;
        plot(timeProfile, Aprofile, 'LineWidth', 1.5);
        ylabel('Contact Area (m^2)');
        yyaxis right;
        plot(timeProfile, Muprofile, 'LineWidth', 1.5);
        ylabel('Effective Friction Coeff (\mu)');
        xlabel('Time (s)'); title('Tire Contact Area & Friction vs Time');
        grid on;

% ==============================================================================================
        
        % 5.3 Powertrain & energy plots
        
        % Kinetic & Potential Energy
        KE  = 0.5 * M_effective * velocityProfile.^2;
        GPE = M_effective * 9.81 * zt(1:length(velocityProfile));
        figure('Name', 'KE & PE'); 
        plot(timeProfile, KE, 'r', timeProfile, GPE, 'b', 'LineWidth', 1.5); 
        xlabel('Time (s)'); ylabel('Energy (Joules)');
        title('Kinetic & Potential Energy');
        legend('Kinetic Energy', 'Potential Energy'); grid on;

        % Electrical & Mechanical Power
        active_ratio  = gearRatios(1) * finalDriveRatio;
        whlrpm        = velocityProfile * (1/wheelRadius);
        motorrpm      = whlrpm * active_ratio * (30/pi);
        inst_Torque   = interp1(rpm_table, T_max_table, motorrpm, 'linear', 0);
        motorPower_kW = (inst_Torque .* motorrpm .* pi ./ 30) ./ 1000;
        
        figure('Name', 'Powertrain: Power Draw vs Time');
        hold on;
        plot(timeProfile, powerProfile / 1000, 'Color', [1 0.5 0], 'LineWidth', 1.5, 'DisplayName', 'Electrical Power (Battery)');
        plot(timeProfile, motorPower_kW, 'm', 'LineWidth', 1.5, 'DisplayName', 'Mechanical Power (Motor)');
        xlabel('Time (s)'); ylabel('Power (kW)');
        title('Powertrain Power Flow Over Lap');
        legend('Location', 'best'); grid on; hold off;

% ==============================================================================================

        % 5.5 Other Debug plots

        % Radius over lap time
        figure('Name', 'Radius Over Lap');
        plot(timeProfile, RProfile(1:end-1), 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontWeight', 'bold'); 
        ylabel('Radius (m)', 'FontWeight', 'bold');
        title('Instantaneous Radius Over Lap', 'FontWeight', 'bold');
        grid on;
        
        % Vehicle time gradient over track layout
        figure('Name', 'Time Gradient Map');
        scatter(xresMCP_laps(1:end-1), yresMCP_laps(1:end-1), 20, timeProfile, 'filled');
        colormap(turbo);
        c = colorbar;
        c.Label.String = 'Time (s)';
        c.Label.FontWeight = 'bold';
        xlabel('X Position (m)', 'FontWeight', 'bold');
        ylabel('Y Position (m)', 'FontWeight', 'bold');
        title('Vehicle Time Gradient Over Track Layout', 'FontWeight', 'bold');
        grid on; axis equal;
        
        % Delta time over lap time
        figure('Name', 'Integration Time Step');
        plot(timeProfile, DTprofile, 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontWeight', 'bold');
        ylabel('dt (s)', 'FontWeight', 'bold');
        title('Integration Time Step (\Delta t) Over Lap', 'FontWeight', 'bold');
        grid on;
        
        % F_CMD over lap time
        figure('Name', 'F_CMD');
        plot(timeProfile, FCMDprofile, 'LineWidth', 1.5);
        xlabel('Time (s)', 'FontWeight', 'bold');
        ylabel('Force (N)', 'FontWeight', 'bold');
        title('Raw Commanded Force (F_{CMD}) Over Lap', 'FontWeight', 'bold');
        ylim([-3000 6000]);
        grid on;

        % Track Banking Plot
        distance_plot = s_track(1:end-1); 
        banking_deg = rad2deg(bankingProfile(1:length(distance_plot)));
        
        plot(distance_plot, banking_deg, 'LineWidth', 1.5);
        xlabel('Distance (m)', 'FontWeight', 'bold'); 
        ylabel('Banking Angle (deg)', 'FontWeight', 'bold');
        title('Track Banking Angle Over Distance', 'FontWeight', 'bold');
        grid on;

        % 1. Calculate the differences
        d_banking = diff(banking_deg);         % Difference in angle (deg)
        d_distance = diff(distance_plot);      % Difference in distance (m)
    
        % 2. Calculate the derivative (rate of change)
        % Resulting units: Degrees per Meter (deg/m)
        banking_rate = d_banking ./ d_distance;

        % 3. Align the distance vector
        % diff() reduces the vector length by 1, so we use the midpoints of the segments
        dist_midpoints = distance_plot(1:end-1) + diff(distance_plot)/2;

        % 4. Plotting
        figure;
        plot(dist_midpoints, banking_rate, 'r', 'LineWidth', 1.2);
        grid on;
        xlabel('Distance (m)', 'FontWeight', 'bold');
        ylabel('Banking Change (deg/m)', 'FontWeight', 'bold');
        title('Rate of Change of Banking Angle', 'FontWeight', 'bold');
        
        % --- 5.1.5 Track Banking Heat Map ---
        figure('Name', 'Track Banking Heat Map');
        
        % Convert the smoothed banking to degrees for the plot
        % Align length with the track coordinates
        banking_map_deg = rad2deg(bankingProfile(1:length(xresMCP_laps)-1));
        
        % Scatter plot using X and Y with banking as the color (C)
        scatter(xresMCP_laps(1:end-1), yresMCP_laps(1:end-1), 30, banking_map_deg, 'filled');
        
        % Formatting
        colormap(turbo); 
        c = colorbar;
        c.Label.String = 'Banking Angle (deg)';
        c.Label.FontWeight = 'bold';
        
        % Set limits slightly wider than your 1.5 deg to see contrast clearly
        clim([-1.8, 1.8]); 
        
        xlabel('X Position (m)', 'FontWeight', 'bold');
        ylabel('Y Position (m)', 'FontWeight', 'bold');
        title('Track Layout: Banking Angle Heat Map', 'FontWeight', 'bold');
        grid on; 
        axis equal; % Critical to keep the track shape realistic
% ==============================================================================================

        % 5.6 Telemetries
        
        distance_plot = s_track(1:end-1); 
        time_export   = timeProfile; 
        v_actual      = velocityProfile;
        v_limit       = VLIMprofile;
        % Steer angle approx. (exact wheelbase needed)
        wheelbase          = 1.4; % [m] 
        steerAngle_mag_rad = (wheelbase .* abs(raw_curvature(1:end-1))) .* cos(abs(leanProfile));
        steerAngle_deg     = rad2deg(steerAngle_mag_rad) .* sign(TSignProfile(1:end-1));
        dz = diff(zt); % Change in elevation
        ds = segmentLengths; % True racing line distance for each segment
        grade_rad = atan2(dz, ds); % True angle of incline
        % Normalised inputs
        accel_input = min(1, max(0, F_applied) ./ max(F_power_array, 0.1)); 
        brake_input = min(1, abs(min(0, F_applied)) / F_brake_wheel_max);
        lean_deg    = rad2deg(leanProfile);
        
        % Telemetry plots
        figure('Name', 'Driver inputs vs Dist');
        
        subplot(5,1,1);
        plot(distance_plot, steerAngle_deg, 'b', 'LineWidth', 1.5);
        ylabel('Steer (deg)', 'FontWeight', 'bold'); 
        title('Driver Telemetry vs Track Distance', 'FontWeight', 'bold'); grid on;
        
        subplot(5,1,2);
        plot(distance_plot, accel_input, 'g', 'LineWidth', 1.5);
        ylabel('Throttle (0-1)', 'FontWeight', 'bold'); 
        ylim([-0.1 1.1]); grid on;
        
        subplot(5,1,3);
        plot(distance_plot, brake_input, 'r', 'LineWidth', 1.5);
        ylabel('Brake (0-1)', 'FontWeight', 'bold'); 
        ylim([-0.1 1.1]); grid on;
        
        subplot(5,1,4);
        plot(distance_plot, lean_deg, 'm', 'LineWidth', 1.5);
        ylabel('Lean (deg)', 'FontWeight', 'bold');
        xlabel('Distance (m)', 'FontWeight', 'bold'); grid on;

        subplot(5,1,5);
        hold on;
        plot(distance_plot, v_limit, 'g--', 'LineWidth', 1.2, 'DisplayName', 'V Limit');
        plot(distance_plot, v_actual, 'r', 'LineWidth', 1.5, 'DisplayName', 'Actual V');
        ylabel('Speed (m/s)', 'FontWeight', 'bold');
        title('Driver Telemetry vs Track Distance', 'FontWeight', 'bold');
        legend('Location', 'northeast'); grid on; hold off;

        telemetry_table = table(distance_plot(:), time_export(:), v_actual(:), v_limit(:), ...
                            steerAngle_deg(:), accel_input(:), brake_input(:), lean_deg(:), grade_rad(:), ...
                            'VariableNames', {'Distance_m', 'Time_s', 'Velocity_ms', 'VLIM_ms', ...
                                              'SteerAngle_deg', 'Throttle_0to1', 'Brake_0to1', 'Lean_deg', 'Grade_rad'});
                                          
        filename_csv = 'Aragon_Telemetry.csv';
        writetable(telemetry_table, filename_csv);
        fprintf('Telemetry exported to %s\n', filename_csv);

% ==============================================================================================
    
end
end