function [trajMCP, trackData] = Sim(~,~) % - Keep as function to allow feasibility tests/multiple running
clear all; clc; close all;
% Debug
debugMode = true; % True for all plots, False for important

% Module calling
filename = 'high_fidel_track.csv';
[trackDataOut] = processTrack(filename);

% Import vars from processTrack module
xresMCP_laps = trackDataOut.xresMCP_laps;
yresMCP_laps = trackDataOut.yresMCP_laps;
segmentLengths = trackDataOut.segmentLengths;
RProfile = trackDataOut.RProfile;
TSignProfile = trackDataOut.TSignProfile;
trajMCP = [xresMCP_laps, yresMCP_laps];
trackData = trackDataOut;
zt = trackDataOut.zt;
finalStepLocs = trackDataOut.finalStepLocs;
scale_factor = trackDataOut.scale_factor;
xin = trackDataOut.xin;
xout = trackDataOut.xout;
yin = trackDataOut.yin;
yout = trackDataOut.yout;
name = trackDataOut.name;
xt = trackDataOut.xt;
yt = trackDataOut.yt;
xresMCP = trackDataOut.xresMCP;
yresMCP = trackDataOut.yresMCP;
bankingProfile = trackDataOut.banking;

% 3. Sanitize Start/Finish Line (Do this AFTER smoothing!)
% We force the start and end to be infinite straights.
% Doing this last ensures the smoothing doesn't "blur" the corner into the start line.
sanitization_window = 20; 
RProfile(1:sanitization_window) = inf; 
RProfile(end-sanitization_window:end) = inf; 

% 4. Final Safety Check (No zeros allowed)
RProfile(RProfile < 0.1) = inf; 
% -------------------------------------------

%% Constants
P_max = 48 * 1000; % Max power in watts (converted from kW to W)
finalDriveRatio = 3.68;
wheelRadius = 0.601/2; % meters
frontalArea = 0.3; % square meters
cd = 0.4; % Drag coefficient
rho = 1.225; % kg/m^3
tireFrictionCoeff = 1; % Maximum friction coefficient
carMass = 220; % kg
h_cog = 0.45; %Cog bike
t_tyre = 67.8/1000; % tyre thickness bike
maxLeanDeg  = 55;
maxLeanRad  = deg2rad(maxLeanDeg);
maxLeanRateDeg  = 30;           % e.g. 60 deg/s, tune as you like
maxLeanRate     = deg2rad(maxLeanRateDeg);  % [rad/s]
g = 9.81; % Gravitational acceleration in m/s²
Ad = 0.004; % Rolling resistance coefficient (velocity-independent)
Bd = 0.000025; % Rolling resistance coefficient (velocity-dependent)
Me_scalingfactor = 1.1;
M_effective = carMass * Me_scalingfactor;
speed_limit = 85;
Fz_prev = M_effective * g;
useLeanRateClamp = true;
useMaxLeanClamp  = true;

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


%% Initialize starting conditions
velocity = 0.1; % Initial velocity in m/s
time = 0; % Initial time
dt = 0.1;
dt_seg = 0.2;

% Define arrays to store velocity and time for plotting
velocityProfile = [];
timeProfile = [];
velocityTrack = []; % Store velocity at each track point for gradient plot
accel = [];
Fcommm = [];
%% Brake Force Solver
% --- Brake system parameters (front wheel only here) ---
rotor_OD = 0.320;              % [m]
rotor_ID = 0.246;              % [m]
R_eff    = 0.5*(rotor_OD + rotor_ID);   % effective pad radius

D_piston = 33.9e-3;            % [m]
N_piston = 4;
A_piston = N_piston * pi*(D_piston/2)^2;  % total piston area

mu_pad   = 0.4;                % pad friction coeff (guess)
lineP_max = 1e6;               % [Pa] ~10 bar, tune to taste

T_brake_max = 2 * mu_pad * lineP_max * A_piston * R_eff;  % factor 2: two pads
F_brake_wheel_max = T_brake_max / wheelRadius;            % [N] at tyre

%% --- NEW: PRE-CALCULATE ROBUST SPEED LIMIT PROFILE (PARANOID MODE) ---
% 1. Geometric Limit (Cornering Speed)
VLIMprofile = zeros(length(xresMCP_laps)-1, 1);
Npts_pre = length(RProfile);

for k = 1:length(xresMCP_laps)-1
    % Look ahead slightly to smooth corner entry
    N_smooth = 5; 
    indices_check = mod((k : k + N_smooth) - 1, Npts_pre) + 1;
    R_win = RProfile(indices_check);
    R_val = R_win(isfinite(R_win) & (R_win > 0));
    
    if isempty(R_val)
        R_k = inf;
    else
        R_k = min(R_val); 
    end
    
    theta_b = abs(bankingProfile(k));
    
    % Physics Geometric Limit
    if isfinite(R_k) && R_k > 0
        mu_corn = tireFrictionCoeff * 0.9; 
        num = mu_corn + tan(theta_b);
        den = 1 - mu_corn * tan(theta_b);
        if den < 0.01, den = 0.01; end
        
        v_corn = sqrt(g * R_k * (num / den));
        VLIMprofile(k) = min(v_corn, speed_limit);
    else
        VLIMprofile(k) = speed_limit;
    end
end

% 2. BACKWARD PASS (The "Paranoid" Braking Curve)
% We calculate the mechanical limits, but we PLAN as if we have weak brakes.
% This compensates for the time it takes to squeeze the lever (Jerk limit).

% A) Limits
a_tire_limit = 9.81 * tireFrictionCoeff;
a_mech_limit = F_brake_wheel_max / M_effective;
a_max_possible = min(a_tire_limit, a_mech_limit);

% B) Safety Factor (LOWERED TO 0.50)
% 0.50 means we plan to use half our braking power. 
% When we actually brake, we use 100%, but starting early ensures we stop.
safety_factor = 1; 

a_brake_plan = a_max_possible * safety_factor; 

fprintf('--- BRAKING LOGIC ---\n');
fprintf('Tire Limit: %.2f m/s^2\n', a_tire_limit);
fprintf('Mech Limit: %.2f m/s^2\n', a_mech_limit);
fprintf('Planning Limit: %.2f m/s^2 (Safety Factor %.2f)\n', a_brake_plan, safety_factor);
fprintf('---------------------\n');

% Run backward pass 3 times
for pass = 1:3
    for k = length(VLIMprofile)-1:-1:1
        dist = segmentLengths(k);
        v_next = VLIMprofile(k+1);
        
        % v_now = sqrt(v_next^2 + 2 * a * d)
        v_brake_limit = sqrt(v_next^2 + 2 * a_brake_plan * dist);
        
        VLIMprofile(k) = min(VLIMprofile(k), v_brake_limit);
    end
    % Wrap around
    v_first = VLIMprofile(1);
    v_last_brake = sqrt(v_first^2 + 2 * a_brake_plan * segmentLengths(end));
    VLIMprofile(end) = min(VLIMprofile(end), v_last_brake);
end

%% Simulation loop with power-sensitive lap time scaling
for i = 1:length(xresMCP_laps)-1  % One less due to diff

    turnSign = TSignProfile(i);
    r_turn = RProfile(i);
    
    % --- 3D PHYSICS: Get Local Banking ---
    theta_bank = bankingProfile(i); % Radians (check processTrack output!)
    % Ensure correct sign: standard convention is positive banking helps the turn
    % We assume 'theta_bank' is always positive magnitude of banking for now
    if sign(theta_bank) == sign(turnSign)
    % Banking helps the turn - use positive value
    theta_bank_effective = abs(theta_bank);
    else
    % Adverse banking - use negative value or set to zero
    theta_bank_effective = -abs(theta_bank);  % Or set to 0 for safety
    end

    % --- velocity-dependent look-ahead size (keep your scaling) ---
    % Simulates rider vision
    % Low speed: Look immediately ahead (Nmin)
    % High speed: Look far ahead to anticipate braking (Nmax)
    Nmin = 1;
    Nmax = 200;
    v_ref = min(speed_limit, maxV);

    % Interpolates look ahead distance based on speed (alpha is ratio 0-1)
    alpha = max(0, min(velocity / v_ref, 1));
    Nlook = round(Nmin + (Nmax - Nmin)*alpha);

    % indices for look-ahead window
    Npts = length(RProfile);
    
    % CYCLIC LOOK-AHEAD: Wrap around to the start of the array
    % This lets the driver see "Turn 1" while approaching the "Finish Line"
    indices_to_check = mod((i : i + Nlook) - 1, Npts) + 1;
    
    % window of radii ahead
    R_window = RProfile(indices_to_check);

    % ignore straights / invalid
    R_valid = R_window(isfinite(R_window) & (R_window > 0));

    if isempty(R_valid)
        R_min_forward = inf;      % no real corner ahead
    else
        R_min_forward = min(R_valid);   % tightest corner in the window
    end

    % local timestep (avoid /0)

    % Drag and rolling resistance
    F_drag = 0.5 * cd * rho * frontalArea * velocity^2;
    F_roll = carMass * g * (Ad + Bd * velocity);

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

    % --- B) Clamp max lean magnitude ---
    if useMaxLeanClamp
        phi_target = max(min(phi_target, +maxLeanRad), -maxLeanRad);
    end

    % === Lean Rate Taper (Stability control) ===
    % As bike approaches maximum lean angle, it becomes harder to lean more
    % Taper the roll rate to 0 as max lean is approached to prevent snapping or overshoot

    % Calculates how close we are to physical limit (0 = upright, 1 = max lean)
    phi_ratio = abs(phi_prev) / (0.7* maxLeanRad); %tune 0.xx value to change taper start point
    phi_ratio = min(max(phi_ratio,0),1);
    
    % Quadratic taper to roll rate
    p = 2;
    taper = 1 - phi_ratio^p;              % taper factor: 1 (full movement) -> 0 (no movement)
    maxLeanRate_eff = maxLeanRate * taper;

    % === Lean Rate Limiting ===
    % Ensures bike cannot roll faster than physically possible (maxLeanRate_eff)
    if useLeanRateClamp
        maxDeltaPhi = maxLeanRate_eff * dt_seg;
        dphi = phi_target - phi_prev;

        if abs(dphi) > maxDeltaPhi
            phi = phi_prev + maxDeltaPhi * sign(dphi);
        else
            phi = phi_target;
        end
    else
        phi = phi_target;
    end

    if i == 1
        leanProfile = zeros(length(xresMCP_laps)-1,1);
    end

    leanProfile(i) = phi;
    phi_prev       = phi;

    %Motorbike Tyre Data
    phi_relative_to_road = max(0, abs(phi) - theta_bank);
    leanDeg_AreaTable = [0 5 10 15 20 25 30 35 40 45 50 55];                     % [deg]
    contactAreaTable  = [0.0204 0.0203 0.0201 ...
        0.0197 0.0192 0.0185 ...
        0.0177 0.0167 0.0156 ...
        0.0144 0.0131 0.0117];     % [m^2]

    A_contact = interp1(leanDeg_AreaTable, contactAreaTable, rad2deg(phi_relative_to_road), 'linear', 'extrap');
    if i == 1, Aprofile = zeros(length(xresMCP_laps)-1,1); end
    Aprofile(i) = A_contact;
    A0 = contactAreaTable(1);
    mu_adjust = ((A_contact)/A0);
    
    % Friction calc (Standard)
    mu_eff = tireFrictionCoeff * mu_adjust;  % Contact area already accounts for lean effects
    if i == 1, Muprofile = zeros(length(xresMCP_laps)-1,1); end
    Muprofile(i) = mu_eff;

    % Lateral force demand
    if r_turn ~= Inf, F_lat = M_effective * velocity^2 / r_turn; else, F_lat = 0; end

    % Elevation
    dz = zt(i+1) - zt(i);
    ds_seg = segmentLengths(i);
    sin_theta = dz / sqrt(ds_seg^2 + dz^2); % Longitudinal slope

    % Normal Force (Approx)
    cos_theta = sqrt(1 - sin_theta^2);
    Fz = M_effective * g * cos(phi) * cos(cos_theta) + M_effective * (velocity^2 / r_turn) * sin(phi);
    F_tire_total = (mu_eff * Fz);
    Fz_prev = Fz;
    if i == 1, FTyreprofile = zeros(length(xresMCP_laps)-1,1); end
    FTyreprofile(i) = F_tire_total;

    % Just reads the pre-calculated limit
    v_limit = VLIMprofile(i);

    % --- Remaining grip available for accel/braking ---
    % Finite amount of grip, calculates how much required to hold turn (F_lat).
    % Remaining grip (F_long_cap) is available for accel/braking.
    % If corner too hard, F_long_cap is 0 so no braking or accel
    F_long_cap = sqrt(max(F_tire_total^2 - F_lat^2, 0));   % >=0

    % --- Power-limit in acceleration only ---
    if velocity > 0
        F_power_limit = P_max / velocity;
    else
        F_power_limit = inf;   % at very low speed power limit isn't binding
    end
  %% --- Driver Decision Logic (Gas vs Brake) ---
    
    
    % how far above the safe corner speed we are
    vdelta = max(0, velocity - v_limit);
    
    % Brake modulation:
    % If slightly overspeed, brake gently. If signif overspeed (>xm/s) full brake.
    brakeBandwidth = 2;   % [m/s], tune this value (speed when 100% braking applied)
    brakeScale = min(1, vdelta / brakeBandwidth);
  % --- Simple "driver": decide whether to gas or brake ---
    % look at how far above / below the local speed limit we are

    % Error-based control (Proportional)
    speed_error = v_limit - velocity;
    Kp = 1500; % Tune this: higher = more aggressive following

    if speed_error > 0
        % Need to speed up: Request force proportional to error, capped by motor
        F_cmd = min(F_power_limit, speed_error * Kp);
    else
        % Need to slow down: Request braking proportional to error
        % Use the existing brakeScale logic or a simple gain
        F_cmd = max(-F_brake_wheel_max, speed_error * Kp) - F_drag - F_roll;
    end

    % --- Apply traction-circle and power/brake limits with correct sign ---
    if F_cmd >= 0
        % accelerating: limited by traction & power
        F_long = min([F_cmd, F_long_cap, F_power_limit]);
    else
        % braking: negative, limited by traction & brake system
        F_long = F_cmd;
    end

    if i == 1
        FCMDprofile = zeros(length(xresMCP_laps)-1,1);
    end

    FCMDprofile(i) = F_cmd;
    
    % Grav force
    F_grav = M_effective * g * sin_theta;

    % Acceleration
    dv = (F_long - F_drag - F_roll - F_grav) / M_effective;

    %  Only taper during acceleration, never during braking
    if dv > 0
        V66 = min(speed_limit, maxV) * 0.95;

        if velocity > V66
            taper = 1 - (velocity / min(speed_limit, maxV))^2;
            taper = max(taper, 0);
            dv = dv * taper;
        end
    end

   % --- NEW ROBUST INTEGRATION (v^2 = u^2 + 2*a*d) ---
    ds = segmentLengths(i);
    
    % 1. Save the velocity at the START of the segment (u)
    v_prev = velocity; 
    
    % 2. Calculate velocity at the END of the segment (v)
    v_squared = v_prev^2 + 2 * dv * ds;
    
    if v_squared < 0
        velocity = 0;
    else
        velocity = sqrt(v_squared);
    end
    
    % Clamp velocity limits
    velocity = max(0.1, min(velocity, min(speed_limit, maxV))); 
    
    % 3. Calculate Time Step using the average of Start and End
    v_avg = (velocity + v_prev) / 2;
    
    % Prevent division by zero if both are ~0 (rare/impossible with clamp, but safe)
    if v_avg < 0.01
        dt_seg = 0.1; 
    else
        dt_seg = ds / v_avg;
    end

    % Store and update
    velocityProfile = [velocityProfile; velocity];
    timeProfile = [timeProfile; time];
    velocityTrack = [velocityTrack; velocity];
    accel = [accel; dv];

    %Lap Time
    ds = segmentLengths(i);  % segment length
    time = time + (ds/velocity);

    if i == 1
        DTprofile = zeros(length(xresMCP_laps)-1,1);
    end

    DTprofile(i) = dt_seg;

    F_power_array(i) = F_power_limit;
    F_remain_array(i) = F_long_cap;
    F_applied(i) = F_long;
end

%% Assign the variables to the workspace (ensuring they are stored)
assignin('base', 'timeProfile', timeProfile);
assignin('base', 'velocityProfile', velocityProfile);
assignin('base', 'accel', accel);
cmdcycle = [timeProfile, FCMDprofile];
assignin('base', 'Fcom', cmdcycle);

mucycle = [timeProfile, Muprofile];
assignin('base', 'mu', mucycle);
drivecycle = [timeProfile, velocityProfile];
assignin('base', 'DC', drivecycle);

%% Display results of sim
avg_speed_ms = 5078 / timeProfile(end);
fprintf('Lap time: %.3f s\n', timeProfile(end));
fprintf('Average vehicle speed: %.2f m/s (%.2f km/h)\n', avg_speed_ms, avg_speed_ms * 3.6);

%% Plots

% 1. 3D Elevation Plot
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

% 2. Plot velocity as a gradient over the track layout
figure;
scatter(xresMCP_laps(1:end-1), yresMCP_laps(1:end-1), 20, velocityTrack, 'filled');
colormap(turbo); % nice modern colormap
c = colorbar;
c.Label.String = 'Velocity (m/s)';
c.Label.FontWeight = 'bold';
c.Label.FontSize = 14;
clim([0 max(velocityTrack)]);  % full range of velocity

xlabel('X Position (m)', 'FontWeight', 'bold', 'FontSize', 14);
ylabel('Y Position (m)', 'FontWeight', 'bold', 'FontSize', 14);
title('Vehicle Velocity Gradient Over Track Layout', 'FontWeight', 'bold', 'FontSize', 16);
grid on;
ax = gca;
ax.FontWeight = 'bold';
ax.FontSize = 14;

% 3. Plot velocity over time
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

%% DEBUG PLOTS (Toggle with debugMode)
if debugMode
    
    figure;
    plot(F_applied, 'k'); hold on;
    plot(F_power_array, 'r--');
    plot(F_remain_array, 'g--');
    legend('Applied Force', 'Power Limit', 'Tire Limit');
    xlabel('Segment Index');
    ylabel('Force (N)');
    title('Force Limiting Conditions');
    ylim([-3000 6000]);

    figure;
    plot(timeProfile, rad2deg(leanProfile))
    legend('Lean Angle')
    xlabel('Time (s)')
    ylabel('Lean Angle (deg)')
    grid on
    title('Lean Angle Over Lap')

    figure;
    plot(timeProfile, Muprofile)
    legend('Contact Area')
    xlabel('Time (s)')
    ylabel('Friction Coeff ')
    grid on
    title('Fricion Coeff Over Lap')

    figure;
    plot(timeProfile, RProfile(1:end-1))
    legend('Instantaneous Radius')
    xlabel('Time (s)')
    ylabel('Meters (m)')
    grid on
    title('Radius Over Lap')

    figure;
    plot(timeProfile, Aprofile)
    legend('Instantaneous contact Area')
    xlabel('Time (s)')
    ylabel('Area (m^2)')
    grid on
    title('contact Area Over Lap')

    figure;
    scatter(xresMCP_laps(1:end-1), yresMCP_laps(1:end-1), 20, timeProfile, 'filled');
    colormap(turbo); % nice modern colormap
    c = colorbar;
    c.Label.String = 'Time (s)';
    c.Label.FontWeight = 'bold';
    c.Label.FontSize = 14;
    clim([min(timeProfile) max(timeProfile)]);  % full range of velocity

    xlabel('X Position (m)', 'FontWeight', 'bold', 'FontSize', 14);
    ylabel('Y Position (m)', 'FontWeight', 'bold', 'FontSize', 14);
    title('Vehicle Time Gradient Over Track Layout', 'FontWeight', 'bold', 'FontSize', 16);
    grid on;
    ax = gca;
    ax.FontWeight = 'bold';
    ax.FontSize = 14;

    figure;
    plot(timeProfile, DTprofile)
    legend('Instantaneous contact Area')
    xlabel('Position')
    ylabel('dt (s)')
    grid on
    title('delta Time Over Lap')

    figure;
    plot(timeProfile, FCMDprofile)
    legend('Force')
    xlabel('Time (s)')
    ylabel('force (N)')
    grid on
    title('F_CMD Over Lap')
    ylim([-3000 6000])

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

    figure;
    plot(finalStepLocs*scale_factor, zt, 'LineWidth', 2);
    xlabel('Distance (m)', 'FontWeight', 'bold', 'FontSize', 14);
    ylabel('Elevation (m)', 'FontWeight', 'bold', 'FontSize', 14);
    title('Track Elevation Profile', 'FontWeight', 'bold', 'FontSize', 16);
    grid on;
    ax = gca;
    ax.FontWeight = 'bold';
    ax.FontSize = 14;

    KE = 0.5 * M_effective * velocityProfile.^2;
    GPE = M_effective * 9.81 * zt(1:length(velocityProfile));
    figure; plot(timeProfile, KE, 'r', timeProfile, GPE, 'b'); legend('Kinetic Energy', 'Potential Energy');

    % Quick check plot for inner/outer
    figure; hold on;
    % plot inner track
    plot(xin,yin,'color','b','linew',2)
    % plot outer track
    plot(xout,yout,'color','r','linew',2)
    title('Inner/Outer Boundaries Check');
    hold off

    %plot minimum curvature trajectory (Detailed Geometry Plot)
    figure;
    hold on;

    % Plot starting line
    plot([xin(1) xout(1)], [yin(1) yout(1)], 'k', 'LineWidth', 2);

    % Plot reference line
    plot(xt, yt, 'k--', 'LineWidth', 1);

    % Plot inner track
    plot(xin, yin, 'r', 'LineWidth', 1);

    % Plot outer track
    plot(xout, yout, 'r', 'LineWidth', 1);

    % Plot minimum curvature path
    plot(xresMCP, yresMCP, 'b', 'LineWidth', 2);

    hold off;

    xlabel('x (m)', 'FontWeight', 'bold', 'FontSize', 14);
    ylabel('y (m)', 'FontWeight', 'bold', 'FontSize', 14);
    title(sprintf(name, '%s - Minimum Curvature Trajectory'), 'FontWeight', 'bold', 'FontSize', 16);
    legend('Starting Line', 'Reference Line', 'Inner Track', 'Outer Track', 'Minimum Curvature Path', ...
        'Location', 'best', 'FontWeight', 'bold', 'FontSize', 12);

    % grid on;

    % Make axis ticks bold and slightly larger
    ax = gca;
    ax.FontWeight = 'bold';
    ax.FontSize = 14;
    trajMCP = [xresMCP yresMCP];

end
end

[trajMCP, trackData] = Sim();