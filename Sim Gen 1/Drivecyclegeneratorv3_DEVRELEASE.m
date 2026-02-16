tic;
clear; 
clc; 
close all;

function [trajMCP, trackDataOut] = Sim(~,~) % - Keep as function to allow feasibility tests/multiple running
%% 1. Basic Cleanup/Init
% Debug
debugMode = true; % True for all plots, False for important

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
bankingProfile      = trackDataOut.banking;

% Link start/finish
% Forces the start and end to be infinite straights to prevent sharp turns/decel.
RProfile(1:20) = inf; 
RProfile(end-20:end) = inf; 
RProfile(RProfile < 0.1) = inf; 

raw_curvature = 1 ./ RProfile;
raw_curvature(isnan(raw_curvature)) = 0;

% Smooth the curvature to remove spikes and make simulation stable (actual driver uses real track)
clean_curvature = smoothdata(raw_curvature, 'gaussian', 10);

% Convert back to radius for VLIM
RProfile_Clean = 1 ./ clean_curvature;

% Clamps Infs and tiny radii
max_straight_R = 100000000;
RProfile_Clean(abs(RProfile_Clean) > max_straight_R) = max_straight_R;
% =========================================================================

% 1.2 Specific Constants
P_max               = 48 * 1000; % Max power in watts (converted from kW to W)
finalDriveRatio     = 3.68;
wheelRadius         = 0.601/2; % meters
frontalArea         = 0.3; % square meters
cd                  = 0.4; % Drag coefficient
rho                 = 1.225; % kg/m^3
tireFrictionCoeff   = 1; % Maximum friction coefficient
carMass             = 220; % kg
h_cog               = 0.45; %Cog bike
t_tyre              = 67.8/1000; % tyre thickness bike
maxLeanDeg          = 55;
maxLeanRad          = deg2rad(maxLeanDeg);
maxLeanRateDeg      = 30;           % e.g. 60 deg/s, tune as you like
maxLeanRate         = deg2rad(maxLeanRateDeg);  % [rad/s]
g                   = 9.81; % Gravitational acceleration in m/s²
Ad                  = 0.004; % Rolling resistance coefficient (velocity-independent)
Bd                  = 0.000025; % Rolling resistance coefficient (velocity-dependent)
Me_scalingfactor    = 1.1;
M_effective         = carMass * Me_scalingfactor;
speed_limit         = 85;
useLeanRateClamp    = true;
useMaxLeanClamp     = true;
leanDeg_AreaTable   = [0 5 10 15 20 25 30 35 40 45 50 55];
contactAreaTable    = [0.0204 0.0203 0.0201 0.0197 0.0192 0.0185 ...
                        0.0177 0.0167 0.0156 0.0144 0.0131 0.0117];

% Power curve data
PPeak_kW = [0, 3.142, 6.283, 9.425, 12.566, 15.708, 18.85, 21.991, 25.133, 28.274, 31.416, ...
    34.558, 37.699, 40.841, 43.982, 47.124, 47.8, 48];
RPM_peakPower = [0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, ...
    3250, 3500, 3750, 4000, 7500];

% Use peak power curve
motorRPM = interp1(PPeak_kW, RPM_peakPower, P_max / 1000, 'linear', 'extrap');
curveType = 'Peak';

% Calculate max velocity
maxV = wheelRadius * 2 * pi * (motorRPM / finalDriveRatio) / 60;

% Display results for power/vel
fprintf('Using %s Power Curve\n', curveType);
fprintf('Motor RPM at P_max = %.1f kW: %.2f RPM\n', P_max / 1000, motorRPM);
fprintf('Estimated max vehicle speed: %.2f m/s (%.2f km/h)\n', ...
    maxV, maxV * 3.6);
% ===========================================================================

% 1.3 Init starting conditions
velocity = 0.1; % Initial velocity in m/s
time = 0; % Initial time

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


% 1.4 Brake Force Solver
% --- Brake system parameters (front wheel only here) ---
rotor_OD            = 0.320;              % [m]
rotor_ID            = 0.246;              % [m]
R_eff               = 0.5*(rotor_OD + rotor_ID);   % effective pad radius

D_piston            = 33.9e-3;            % [m]
N_piston            = 4;
A_piston            = N_piston * pi*(D_piston/2)^2;  % total piston area

mu_pad              = 0.4;                % pad friction coeff (guess)
lineP_max           = 1e6;               % [Pa] ~10 bar, tune to taste

T_brake_max         = 1 * mu_pad * lineP_max * A_piston * R_eff;  % factor 2: two pads
F_brake_wheel_max   = T_brake_max / wheelRadius;            % [N] at tyre

%% 2. VLIM precalc 
leanRad_AreaTable = deg2rad(leanDeg_AreaTable);
A0 = contactAreaTable(1);
mu_available_curve = tireFrictionCoeff * (contactAreaTable / A0);
mu_required_curve = tan(leanRad_AreaTable);
diff_curve = mu_available_curve - mu_required_curve; %grip margin
interp_angles = linspace(0, deg2rad(55), 1000);
diff_interp = interp1(leanRad_AreaTable, diff_curve, interp_angles, 'pchip');
[~, idx_cross] = min(abs(diff_interp)); 
limit_lean_angle = interp_angles(idx_cross);
mu_at_limit = interp1(leanRad_AreaTable, mu_available_curve, limit_lean_angle, 'pchip');
mu_corn_dynamic = mu_at_limit * 0.98;

% 2.1 Geometric limit (cornering speed)
VLIMprofile = zeros(length(xresMCP_laps)-1, 1);
for k = 1:length(xresMCP_laps)-1
    R_k = abs(RProfile_Clean(k));    
    if isfinite(R_k) && R_k > 0 && R_k < 10000
        % Camber direction
        turnSign = TSignProfile(k);
        theta_bank = bankingProfile(k);
        
        % If Turn and Camber have same sign -> Good (+)
        % If different signs -> Bad (-)
        if sign(theta_bank) == sign(turnSign)
             theta_bank_effective = abs(theta_bank);
        else
             theta_bank_effective = -abs(theta_bank);
        end

        % Tire friction limit, speed at lose grip (with camber/bank)
        num = mu_corn_dynamic + tan(theta_bank_effective);
        den = 1 - mu_corn_dynamic * tan(theta_bank_effective);
        if den < 0.01, den = 0.01; end %safety prevents div by 0
        v_friction = sqrt(g * R_k * (num / den));
        
        % hard lean limit (geometric)
        real_max_lean_rad = min(maxLeanRad, limit_lean_angle);
        effective_max_lean = real_max_lean_rad + theta_bank_effective;
        if effective_max_lean >= pi/2, effective_max_lean = pi/2 - 0.01; end
        v_lean_geometry = sqrt(g * R_k * tan(effective_max_lean));
        
        % final limit
        VLIMprofile(k) = min([v_friction, v_lean_geometry, speed_limit]);
    else
        VLIMprofile(k) = speed_limit;
    end
end

% 2.2 BACKWARD PASS
a_mech_limit = F_brake_wheel_max / M_effective; 
safety_factor = 1.0; % just in case, was used before but not needed now
for pass = 1:3
    for k = length(VLIMprofile)-1:-1:1
        dist = segmentLengths(k);
        v_next = VLIMprofile(k+1);
        
        % --- GEOMETRY & LEAN ---
        R_here = abs(RProfile_Clean(k));
        turnSign = TSignProfile(k);
        theta_bank = bankingProfile(k);
        
        if sign(theta_bank) == sign(turnSign)
             theta_bank_effective = abs(theta_bank);
        else
             theta_bank_effective = -abs(theta_bank);
        end
        
        % Estimates lean angle
        if isfinite(R_here) && R_here > 0 && R_here < 10000
             phi_flat = atan(v_next^2 / (g * R_here));
             phi = phi_flat - theta_bank_effective;
        else
             phi = 0;
        end
        
        % --- FRICTION PENALTY --- (from table before)
        phi_relative_to_road = abs(phi);
        A_contact = interp1(leanDeg_AreaTable, contactAreaTable, ...
                            rad2deg(phi_relative_to_road), 'linear', 'extrap');
        Aprofile(k) = A_contact;
        mu_adjust = (A_contact / A0);
        mu_eff = tireFrictionCoeff * mu_adjust;
        
        % --- TRACTION CIRCLE ---
        % Max deceleration the TIRE can generate (Force/M_eff)

        f_friction = carMass * 9.81 * mu_eff; 
        a_limit_decel_max = f_friction / M_effective;

        % Lateral acceleration required (geometric)
        a_lat_geometric = v_next^2 / max(1, R_here);
        a_banking_assist = 9.81 * tan(theta_bank_effective);
        a_lat_demand_tire = abs(a_lat_geometric - a_banking_assist);
        f_lat_demand = carMass * a_lat_demand_tire;
        a_lat_scaled = f_lat_demand / M_effective;
    
        % if lateral force required is greater than can provide, force speed down
        if a_lat_scaled >= a_limit_decel_max
            num = mu_eff + tan(theta_bank_effective);
            den = 1 - mu_eff * tan(theta_bank_effective);
        if den < 0.01, den = 0.01; end
     
            v_physics_limit = sqrt(9.81 * R_here * (num / den));
     
            VLIMprofile(k) = min(VLIMprofile(k), v_physics_limit);
     
            v_next = VLIMprofile(k); 
            a_grip_available = 0;
        else
             a_grip_available = sqrt(a_limit_decel_max^2 - a_lat_scaled^2); %longit grip available
        end
        
        a_brake_limit_local = min(a_mech_limit, a_grip_available);

        % --- INTEGRATE BACKWARDS ---
        
        % Aero drag
        F_aero = 0.5 * rho * cd * frontalArea * v_next^2;
        a_aero = F_aero / M_effective;
        
        % Rolling resist
        F_roll = carMass * 9.81 * (Ad + Bd * v_next);
        a_roll = F_roll/M_effective;

        % Gravity
        dz = zt(k+1) - zt(k);
        sin_theta = dz / sqrt(dist^2 + dz^2);

        f_grav = carMass * 9.81 * sin_theta;
        a_grav = f_grav/M_effective; 

        % Total decel
        a_total_decel = (a_brake_limit_local * safety_factor) + a_aero + a_roll + a_grav;
        
        if a_total_decel < 0.01, a_total_decel = 0.01; end
        
        v_brake_limit = sqrt(v_next^2 + 2 * a_total_decel * dist);
        VLIMprofile(k) = min(VLIMprofile(k), v_brake_limit);
    end
    
    v_first = VLIMprofile(1);
    v_last_brake = sqrt(v_first^2 + 2 * (a_mech_limit * safety_factor) * segmentLengths(end));
    VLIMprofile(end) = min(VLIMprofile(end), v_last_brake);
end
% ==============================================================================================

%% 3. Simulation loop with power-sensitive lap time scaling
for i = 1:length(xresMCP_laps)-1  % One less due to diff

    turnSign = TSignProfile(i);
    r_turn = RProfile(i);
    

    % Elevation
    dz = zt(i+1) - zt(i);
    ds_seg = segmentLengths(i);
    sin_theta = dz / sqrt(ds_seg^2 + dz^2); % Longitudinal slope
    cos_theta = sqrt(1 - sin_theta^2);
    
    % 3.1 Camber & Roll angle
    % --- Get Local camber ---
    theta_bank = bankingProfile(i); % Radians
    phi_relative_to_road =abs(phi);

    if sign(theta_bank) == sign(turnSign)
    theta_bank_effective = abs(theta_bank);
    else
    theta_bank_effective = -abs(theta_bank);
    end

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
        phi_delta = asin( (t_tyre * sin(phi_i)) / (h_cog - t_tyre) );
    else
        phi_delta = 0;
    end

    phi_mag = phi_i + phi_delta;   % magnitude of effective roll, always ≥0

    if turnSign > 0        % say this corresponds to a LEFT turn geometrically
        phi_signed_target = -phi_mag;   % left = negative
    elseif turnSign < 0    % RIGHT turn
        phi_signed_target =  phi_mag;   % right = positive
    else
        phi_signed_target = 0;
    end

    if i == 1
        phi_prev = 0;
    end

    phi_target = phi_signed_target;   % from previous section

    % --- Clamp max lean magnitude ---
    if useMaxLeanClamp
        phi_target = max(min(phi_target, +maxLeanRad), -maxLeanRad);
    end

    % --- Lean rate taper ---
    % Taper the roll rate to 0 as max lean is approached to prevent snapping or overshoot

    % Define where tapering starts (e.g., at 70% of max lean)
    taper_start_angle = 0.7 * maxLeanRad;

    if abs(phi_prev) < taper_start_angle
        taper = 1; % Full roll rate available
    else
    % Map the current angle from [Start -> Max] to [0 -> 1]
        ratio_in_zone = (abs(phi_prev) - taper_start_angle) / (maxLeanRad - taper_start_angle);
        ratio_in_zone = max(0, min(ratio_in_zone, 1));
    % Taper goes from 1 down to 0
        taper = 1 - ratio_in_zone^2; 
    end            
    maxLeanRate_eff = maxLeanRate * taper;

    % ---Lean Rate Limiting ---
    % Ensures bike cannot roll faster than physically possible (maxLeanRate_eff)
    % maxDeltaPhi = [rad/s] * [m] / [m/s] = [rad]
    if useLeanRateClamp
        maxDeltaPhi = maxLeanRate_eff * segmentLengths(i) / max(velocity, 1);
        dphi = phi_target - phi_prev;

        if abs(dphi) > maxDeltaPhi
            phi = phi_prev + maxDeltaPhi * sign(dphi);
        else
            phi = phi_target;
        end
    else
        phi = phi_target; % only unreachable since useLeanRateClamp is true
    end

    if i == 1
        leanProfile = zeros(length(xresMCP_laps)-1,1);
    end

    leanProfile(i) = phi;
    phi_prev       = phi;
    
    % Friction calc
    A_contact = interp1(leanDeg_AreaTable, contactAreaTable, rad2deg(phi_relative_to_road), 'linear', 'extrap');
    Aprofile(i) = A_contact;
    mu_adjust = ((A_contact)/A0);
    mu_eff = tireFrictionCoeff * mu_adjust;  % Contact area already accounts for lean effects
    if i == 1, Muprofile = zeros(length(xresMCP_laps)-1,1); end
    Muprofile(i) = mu_eff;

    % =====================================================================

    % 3.2 Forces demand

    % Lateral force
    if r_turn ~= Inf 
        F_centrifugal_out = carMass * velocity^2 / r_turn * cos(theta_bank_effective);
        F_gravity_in = carMass * g * sin(theta_bank_effective);
        F_lat_demand = abs(F_centrifugal_out - F_gravity_in);
    else 
        F_lat_demand = 0; 
    end

    % Normal Force
    % Gravity + Banking + Vertical Curvature
    Fz = (carMass * g * cos(theta_bank_effective) * cos_theta) + ...
         (carMass * (velocity^2 / r_turn) * sin(theta_bank_effective));
    
    % Ensure Fz is never negative (prevents imaginary numbers in sqrt)
    Fz = max(0.1, Fz); 

    % 3.3 Traction Limit

    % Total available friction capacity
    F_tire_total = mu_eff * Fz; 
    FTyreprofile(i) = F_tire_total;

    % reads pre-calc limit
    v_limit = VLIMprofile(i);

    % Remaining grip available for accel/braking, calculates how much required to hold turn (F_lat).
    if F_lat_demand > F_tire_total
        % The tire cannot hold the turn even with 0 braking/gas. 
        F_long_cap = 0; 
    else
        F_long_cap = sqrt(F_tire_total^2 - F_lat_demand^2);
    end

    % Power-limit in acceleration only
    if velocity > 0
        F_power_limit = P_max / velocity;
    else
        F_power_limit = inf;   % at very low speed power limit isn't binding
    end

    % 3.4 Driver decision logic
    % Decides whether to gas or brake (bang bang controller)
    % looks at how far above / below the local speed limit we are

    % Init conditions
    vdelta = max(0, velocity - v_limit);
    brakeBandwidth = 0.5;   % [m/s], tune this value
    brakeScale = min(1, vdelta / brakeBandwidth);
    
    if velocity < v_limit
        % ACCELERATION
        F_cmd = F_power_limit;          % try to use all available power
    elseif velocity > v_limit % + margin
        % BRAKING
        F_cmd = -F_brake_wheel_max * brakeScale; % full break request
    end

    % --- Apply traction-circle and power/brake limits with correct sign ---
    if F_cmd >= 0
        % accelerating: limited by traction & power
        F_long = min([F_cmd, F_long_cap, F_power_limit]);
    else
        % braking: negative, limited by traction & brake system
        F_long = F_cmd; %max(F_cmd, -F_long_max_brake);  % most negative allowed, use F_cmd only as 1% overshoot breaks whole simulation
    end

    FCMDprofile(i) = F_cmd;

    % 3.5 Applying final forces

    % Init
    F_drag = 0.5 * cd * rho * frontalArea * velocity^2;
    F_roll = carMass * g * (Ad + Bd * velocity);
    ds = segmentLengths(i);
    v_start = velocity;
    V66 = min(speed_limit, maxV) * 0.95;
    F_long_taper = F_long;
    
    %  Only taper during acceleration, never during braking
    if v_start > V66 && F_long > 0
        ratio = (v_start - V66) / (min(speed_limit, maxV) - V66);
        taper = 1 - ratio;
        taper = max(0, taper);
        F_long_taper = F_long * taper;
    end
    
    % Net Force after taper
    F_grav = carMass * g * sin_theta;
    F_net_start = F_long_taper - F_drag - F_roll - F_grav;
    a_start = F_net_start / M_effective;

    % Predictor step to estimate v_end for a better drag average
    v_end_est = sqrt(max(0.1, v_start^2 + 2 * a_start * ds));
    F_drag_end = 0.5 * cd * rho * frontalArea * v_end_est^2;

    % Acceleration calc
    F_net_avg = F_long_taper - (F_drag + F_drag_end)/2 - F_roll - F_grav; %avg drag as drag depends on velocity (heun's)
    dv_avg = F_net_avg / M_effective;

    % Final velocity for this step
    v_sq = v_start^2 + 2 * dv_avg * ds;
    velocity = sqrt(max(0.1, v_sq));

    % Time step based on average velocity
    v_mean = (v_start + velocity) / 2;
    dt_seg = ds / max(0.1, v_mean);
    % =========================================================================

    % 3.6 Storing & Updating
    velocityProfile(i) = velocity;
    timeProfile(i)     = time;
    accel(i)           = dv_avg;
    DTprofile(i) = dt_seg;
    F_power_array(i) = F_power_limit;
    F_remain_array(i) = F_long_cap;
    F_applied(i) = F_long;

    %Lap Time
    time = time + dt_seg;
end

%% 4. Assign the variables to the workspace (ensuring they are stored)
assignin('base', 'timeProfile', timeProfile);
assignin('base', 'velocityProfile', velocityProfile);
assignin('base', 'accel', accel);
cmdcycle = [timeProfile, FCMDprofile];
assignin('base', 'Fcom', cmdcycle);

mucycle = [timeProfile, Muprofile];
assignin('base', 'mu', mucycle);
drivecycle = [timeProfile, velocityProfile];
assignin('base', 'DC', drivecycle);

%Display results of sim
avg_speed_ms = 5078 / timeProfile(end);
fprintf('Lap time: %.3f s\n', timeProfile(end));
fprintf('Average vehicle speed: %.2f m/s (%.2f km/h)\n', avg_speed_ms, avg_speed_ms * 3.6);

%% 5. Plots

% 5.1 3D Elevation Plot
X_surf = [xin(:)'; xout(:)'];
Y_surf = [yin(:)'; yout(:)'];
Z_surf = [zt(:)'; zt(:)']; 
C_surf = Z_surf; 

figure;
s = surf(X_surf, Y_surf, Z_surf, C_surf);

s.EdgeColor = 'none';    
s.FaceColor = 'interp'; 
colorbar;              
title('3D Track');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal;             
view(3); 

% 5.2 Plot velocity as a gradient over the track layout
figure;
scatter(xresMCP_laps(1:end-1), yresMCP_laps(1:end-1), 20, velocityProfile, 'filled');
colormap(turbo); % nice modern colormap
c = colorbar;
c.Label.String = 'Velocity (m/s)';
c.Label.FontWeight = 'bold';
c.Label.FontSize = 14;
clim([0 max(velocityProfile)]);  % full range of velocity

xlabel('X Position (m)', 'FontWeight', 'bold', 'FontSize', 14);
ylabel('Y Position (m)', 'FontWeight', 'bold', 'FontSize', 14);
title('Vehicle Velocity Gradient Over Track Layout', 'FontWeight', 'bold', 'FontSize', 16);
grid on;
ax = gca;
ax.FontWeight = 'bold';
ax.FontSize = 14;

% 5.3 Plot velocity over time
figure;
plot(timeProfile, velocityProfile, 'LineWidth', 2);
hold on;
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 14);
ylabel('Velocity (m/s)', 'FontWeight', 'bold', 'FontSize', 14);
title('Vehicle Velocity Profile', 'FontWeight', 'bold', 'FontSize', 16);
legend('Velocity', 'FontWeight', 'bold', 'FontSize', 12, 'Location', 'best');
grid on;
ax = gca;
ax.FontWeight = 'bold';
ax.FontSize = 14;
hold off;

%% 6. DEBUG PLOTS (Toggle with debugMode)
if debugMode
    
    % 6.1 Force limiting conditions
    figure;
    plot(F_applied, 'k'); hold on;
    plot(F_power_array, 'r--');
    plot(F_remain_array, 'g--');
    legend('Applied Force', 'Power Limit', 'Tire Limit');
    xlabel('Segment Index');
    ylabel('Force (N)');
    title('Force Limiting Conditions');
    ylim([-3000 6000]);
    
    % 6.2 Lean angle over lap time
    figure;
    plot(timeProfile, rad2deg(leanProfile))
    legend('Lean Angle')
    xlabel('Time (s)')
    ylabel('Lean Angle (deg)')
    grid on
    title('Lean Angle Over Lap')

    % 6.3 Friction coefficient over lap time
    figure;
    plot(timeProfile, Muprofile)
    legend('Contact Area')
    xlabel('Time (s)')
    ylabel('Friction Coeff ')
    grid on
    title('Fricion Coeff Over Lap')
    
    % 6.4 Radius over lap time (how long each corner is)
    figure;
    plot(timeProfile, RProfile(1:end-1))
    legend('Instantaneous Radius')
    xlabel('Time (s)')
    ylabel('Meters (m)')
    grid on
    title('Radius Over Lap')
    
    % Tire contact area over lap time
    figure;
    plot(timeProfile, Aprofile)
    legend('Instantaneous contact Area')
    xlabel('Time (s)')
    ylabel('Area (m^2)')
    grid on
    title('contact Area Over Lap')
    
    % Vehicle time gradient over track layout
    figure;
    scatter(xresMCP_laps(1:end-1), yresMCP_laps(1:end-1), 20, timeProfile, 'filled');
    colormap(turbo);
    c = colorbar;
    c.Label.String = 'Time (s)';
    c.Label.FontWeight = 'bold';
    c.Label.FontSize = 14;
    clim([min(timeProfile) max(timeProfile)]);

    xlabel('X Position (m)', 'FontWeight', 'bold', 'FontSize', 14);
    ylabel('Y Position (m)', 'FontWeight', 'bold', 'FontSize', 14);
    title('Vehicle Time Gradient Over Track Layout', 'FontWeight', 'bold', 'FontSize', 16);
    grid on;
    ax = gca;
    ax.FontWeight = 'bold';
    ax.FontSize = 14;
    
    % delatime time over lap time
    figure;
    plot(timeProfile, DTprofile)
    legend('Instantaneous contact Area')
    xlabel('Position')
    ylabel('dt (s)')
    grid on
    title('delta Time Over Lap')
    
    % Force demanded by computer over lap time
    figure;
    plot(timeProfile, FCMDprofile)
    legend('Force')
    xlabel('Time (s)')
    ylabel('force (N)')
    grid on
    title('F_CMD Over Lap')
    ylim([-3000 6000])
    
    % Actual speed and max possible speed limit over lap time
    figure;
    hold on
    plot(timeProfile, velocityProfile, 'r')
    plot(timeProfile, VLIMprofile, 'g')
    legend('Instantaneous contact Area')
    xlabel('time (s)')
    ylabel('speed (ms)')
    legend('Vehicle Speed', 'Speed Limit')
    grid on
    title('Velocit and vLIM Over Lap')
    
    % Track elevation over distance
    figure;
    plot(finalStepLocs*scale_factor, zt, 'LineWidth', 2);
    xlabel('Distance (m)', 'FontWeight', 'bold', 'FontSize', 14);
    ylabel('Elevation (m)', 'FontWeight', 'bold', 'FontSize', 14);
    title('Track Elevation', 'FontWeight', 'bold', 'FontSize', 16);
    grid on;
    ax = gca;
    ax.FontWeight = 'bold';
    ax.FontSize = 14;
    
    % Kinetic & Potential energy over lap time
    KE = 0.5 * M_effective * velocityProfile.^2;
    GPE = M_effective * 9.81 * zt(1:length(velocityProfile));
    figure; plot(timeProfile, KE, 'r', timeProfile, GPE, 'b'); legend('Kinetic Energy', 'Potential Energy');
    
    % Camber and turning sign over segment
    figure;
    plot(TSignProfile, 'DisplayName', 'Turn Sign');
    hold on;
    bank_deg = rad2deg(bankingProfile);
    plot(bank_deg, 'DisplayName', 'Banking Angle');
    legend;
    title('camber and/turning sign');
    xlabel('Segment Index');
    ylabel('Sign/Angle');

    % Inner outer boundary plot
    figure; hold on;
    plot(xin,yin,'color','b','linew',2)
    plot(xout,yout,'color','r','linew',2)
    title('Inner/Outer Boundaries Check');
    hold off

    % Plot minimum curvature target and whole track
    figure;
    hold on;

    plot([xin(1) xout(1)], [yin(1) yout(1)], 'k', 'LineWidth', 2);
    plot(xt, yt, 'k--', 'LineWidth', 1);
    plot(xin, yin, 'r', 'LineWidth', 1);
    plot(xout, yout, 'r', 'LineWidth', 1);
    plot(xresMCP, yresMCP, 'b', 'LineWidth', 2);

    hold off;

    xlabel('x (m)', 'FontWeight', 'bold', 'FontSize', 14);
    ylabel('y (m)', 'FontWeight', 'bold', 'FontSize', 14);
    title(sprintf(name, '%s - Minimum Curvature Trajectory'), 'FontWeight', 'bold', 'FontSize', 16);
    legend('Starting Line', 'Reference Line', 'Inner Track', 'Outer Track', 'Minimum Curvature Path', ...
        'Location', 'best', 'FontWeight', 'bold', 'FontSize', 12);
end
end

[trajMCP, trackDataOut] = Sim();
elapsedTime = toc;
fprintf('Completed in %.4f seconds.\n', elapsedTime);