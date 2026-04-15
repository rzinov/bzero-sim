function [trajMCP, trackData] = DRIVE_CYCLE_GEN(~,~)
clear
clc

%% Processing  track data
track = readmatrix('nordschleife_xy_limits.csv');
name = 'Nurburgring';

% track data - first point repeated
data = track;

% x,y and track width data
x =  data(:,1);
y =  data(:,2);
twr = data(:,3);
twl = data(:,4);

% interpolate data to get finer curve with equal distances between each segment

% higher no. of segments causes trajectory to follow the reference line
nseg = 3500;

pathXY = [x y];
stepLengths = sqrt(sum(diff(pathXY,[],1).^2,2));
stepLengths = [0; stepLengths]; % add the starting point
cumulativeLen = cumsum(stepLengths);
finalStepLocs = linspace(0,cumulativeLen(end), nseg);
finalPathXY = interp1(cumulativeLen, pathXY, finalStepLocs);
xt = finalPathXY(:,1);
yt = finalPathXY(:,2);
twrt = interp1(cumulativeLen, twr, finalStepLocs,'spline')';
twlt = interp1(cumulativeLen, twl, finalStepLocs,'spline')';

% normal direction for each vertex
dx = gradient(xt);
dy = gradient(yt);
dL = hypot(dx,dy);

% offset curve - anonymous function
xoff = @(a) -a*dy./dL + xt;
yoff = @(a)  a*dx./dL + yt;

% plot reference line
plot(xt,yt,'k')
hold on

% offset data
offset = [-twrt twlt];
for i = 1:numel(xt)
    xin = xoff(offset(i,1));      % get inner offset curve
    yin = yoff(offset(i,1));

    xout  = xoff(offset(i,2));      % get outer offset curve
    yout  = yoff(offset(i,2));
end

% plot inner track
plot(xin,yin,'color','b','linew',2)
% plot outer track
plot(xout,yout,'color','r','linew',2)
hold off

xlabel('x(m)','fontweight','bold','fontsize',14)
ylabel('y(m)','fontweight','bold','fontsize',14)
title(sprintf(name),'fontsize',16)
legend;


% form delta matrices
delx = xout - xin;
dely = yout - yin;

trackData = [xt yt xin yin xout yout];

%% Matrix Definition

% number of segments
n = numel(delx);

% preallocation
H = zeros(n);
B = zeros(size(delx)).';

% formation of H matrix (nxn)
for i=2:n-1

    % first row
    H(i-1,i-1) = H(i-1,i-1) + delx(i-1)^2         + dely(i-1)^2;
    H(i-1,i)   = H(i-1,i)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
    H(i-1,i+1) = H(i-1,i+1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);

    %second row
    H(i,i-1)   = H(i,i-1)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
    H(i,i)     = H(i,i )    + 4*delx(i)^2         + 4*dely(i)^2;
    H(i,i+1)   = H(i,i+1)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);

    % third row
    H(i+1,i-1) = H(i+1,i-1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);
    H(i+1,i)   = H(i+1,i)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);
    H(i+1,i+1) = H(i+1,i+1) + delx(i+1)^2         + dely(i+1)^2;

end

% formation of B matrix (1xn)
for i=2:n-1

    B(1,i-1) = B(1,i-1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i-1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i-1);
    B(1,i)   = B(1,i)   - 4*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i)   - 4*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i);
    B(1,i+1) = B(1,i+1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i+1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i+1);

end

% define constraints
lb = zeros(n,1);
ub = ones(size(lb));

% if start and end points are the same
Aeq      =   zeros(1,n);
Aeq(1)   =   1;
Aeq(end) =   -1;
beq      =   0;

%% Solver

options = optimoptions('quadprog','Display','iter');
[resMCP,~,~,~] = quadprog(2*H,B',[],[],Aeq,beq,lb,ub,[],options);

%% Plotting results

% co-ordinates for the resultant curve
xresMCP = zeros(size(xt));
yresMCP = zeros(size(xt));

for i = 1:numel(xt)
    xresMCP(i) = xin(i)+resMCP(i)*delx(i);
    yresMCP(i) = yin(i)+resMCP(i)*dely(i);
end

%plot minimum curvature trajectory
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
trajMCP = [xresMCP yresMCP]; %xy is generated elevation must be after

disp(size(trajMCP))

%% Corner Radius Solver

% ---- Corner radius for a single lap ----
nLap = length(xresMCP);
RProfileLap  = zeros(nLap,1);
TSignLap     = zeros(nLap,1);
ARProfileLap = zeros(nLap,1);

for i = 1:nLap
    % neighbours within a single lap
    if i == 1
        u1 = 1;    u2 = 2;    u3 = 3;
    elseif i == nLap
        u1 = nLap-2; u2 = nLap-1; u3 = nLap;
    else
        u1 = i-1;  u2 = i;    u3 = i+1;
    end

    x1 = xresMCP(u1);  y1 = yresMCP(u1);
    x2 = xresMCP(u2);  y2 = yresMCP(u2);
    x3 = xresMCP(u3);  y3 = yresMCP(u3);

    a = hypot(x2-x3, y2-y3);
    b = hypot(x1-x3, y1-y3);
    c = hypot(x1-x2, y1-y2);

    area2 = (x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2));
    A = 0.5*abs(area2);

    if A > 0
        RProfileLap(i)    = (a*b*c)/(4*A);
        turnSign(i) = sign(area2);
    else
        RProfileLap(i)    = inf;
        turnSign(i) = 0;
    end
end

%% Drive Cycle Generator Path
% Repeat trajectory
nLaps = 1;
xresMCP_laps = repmat(xresMCP, nLaps, 1);
yresMCP_laps = repmat(yresMCP, nLaps, 1);

% Tile curvature profiles (no fake corner at lap seams)
RProfile    = repmat(RProfileLap,  nLaps, 1);
TSignProfile= repmat(TSignLap,     nLaps, 1);
ARProfile   = repmat(ARProfileLap, nLaps, 1);

% Compute current lap length
dx_seg = diff(xresMCP_laps);
dy_seg = diff(yresMCP_laps);
segmentLengths = sqrt(dx_seg.^2 + dy_seg.^2);
lapLength = sum(segmentLengths);

% Scale to target
targetLap = 20.84 * 1000 * nLaps;  % meters
scale_factor = targetLap / lapLength;

% Apply scaling
xresMCP_laps = xresMCP_laps * scale_factor;
yresMCP_laps = yresMCP_laps * scale_factor;
trajMCP = [xresMCP_laps yresMCP_laps];  % update

% Recompute lengths
dx_seg = diff(xresMCP_laps);
dy_seg = diff(yresMCP_laps);
segmentLengths = sqrt(dx_seg.^2 + dy_seg.^2);

%% Constants
Pmax_Front = 160e3; % Front power 160kW maxing at 40% RPM
Pmax_Rear = 600e3; % Rear power 600kW maxing at 40% RPM
Pmax_Tot = Pmax_Front + Pmax_Rear; % 760kW Max power in watts (converted from kW to W) 1019hp between 2 AC motors, 760kW w/ boost, 580kW output
RPM_max = 14000; % Max RPM of motors 14000
RPM_peak_frac = 0.4; % Peak power reaached at around 40%
RPM_peak = RPM_max * RPM_peak_frac; % RPM at peak power
nPts = 100; % RPM measuring points
RPM = linspace(0, RPM_max, nPts)'; % RPM intervals
gearRatio = 1;
finalDriveRatio = 8; % update
wheelRadius = 0.35; % meters %21inch 0.5334m Taycan Website
frontalArea = 2.32; % square meters %2.35m^2 estimated value
cd = 0.29; % Drag coefficient, Taycan is low drag
rho = 1.225; % kg/m^3
tireFrictionCoeff = 1.45; % Maximum friction coefficient %1.8, average for longitudinal and lateral
carMass = 2350; % kg %2350
h_cog = 0.45; %Cog Taycan, max height 1.378m, estimated
t_tyre = 285/1000; % tyre thickness average of 265 + 305 mm
g = 9.81; % Gravitational acceleration in m/s²
Ad = 0.012; % Rolling resistance coefficient (velocity-independent) Endurance 0.012 Qualifying 0.010
Bd = 0.000025; % Rolling resistance coefficient (velocity-dependent)
Me_scalingfactor = 1.04; % To be optimised
M_effective = carMass * Me_scalingfactor;
speed_limit = 80.56; % Taycan top speed 290 km/h in m/s
Fz_prev = M_effective * g;
v_grid = linspace(0, speed_limit, 290)'; % Vehicle speed limits

% RPM conversion
RPM_wheel  = (v_grid / wheelRadius) * (60/(2*pi));
RPM_motor  = RPM_wheel * finalDriveRatio * gearRatio;
RPM_motor  = min(max(RPM_motor, 0), RPM_max); 

% Power curve smoothstep
smoothstep = @(x) x.^2 .* (3 - 2*x);

% Power curve build
makePowerCurve = @(rpmVec, Pmax, rpmPk) Pmax .* ((rpmVec <= rpmPk) .* smoothstep(max(0, min(1, rpmVec./rpmPk))) + (rpmVec >  rpmPk) .* 1);
% Power separation between motors
P_front_W = makePowerCurve(RPM, Pmax_Front, RPM_peak);
P_rear_W  = makePowerCurve(RPM, Pmax_Rear,  RPM_peak);
P_total_W = P_front_W + P_rear_W;

% Available power from motors


% RPM Plot
figure; hold on; grid on;
plot(RPM, P_front_W/1000, 'LineWidth', 2);
plot(RPM, P_rear_W/1000,  'LineWidth', 2);
plot(RPM, P_total_W/1000, 'LineWidth', 2);
xlabel('Motor speed (RPM)'); ylabel('Power (kW)');
legend('Front','Rear','Total','Location','best');
title('Power curves: peak reached at 40% RPM');

% Display results
curveType = 'Peak@40%RPM';
fprintf('Using %s Power Curve\n', curveType);
fprintf('Motor RPM at P_max = %.1f kW: %.2f RPM\n', Pmax_Tot / 1000, RPM_peak);
fprintf('Estimated max vehicle speed: %.2f m/s (%.2f km/h)\n', ...
    speed_limit, speed_limit * 3.6);

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
%% Brake Force 
% --- Brake system parameters (front) ---
rotor_OD = 0.42;              % [m] 0.42 front 0.41 rear
R_eff    = 0.4*(rotor_OD);   % effective pad radius

D_piston = 44e-3;            % [m]
N_piston = 10;   % 4 pistons per brake 
A_piston = N_piston * pi*(D_piston/2)^2;  % total piston area

mu_pad   = 0.45;                % pad friction coeff (guess)
lineP_max = 1e6;               % [Pa] ~10 bar, tune to taste

T_brake_max = 2 * mu_pad * lineP_max * A_piston * R_eff;  % factor 2: two pads
F_brake_wheel_max_front = T_brake_max / wheelRadius;            % [N] at tyre

% --- Brake system parameters (rear) ---
rotor_OD = 0.41;              % [m] 0.42 front 0.41 rear
R_eff    = 0.4*(rotor_OD);   % effective pad radius

D_piston = 36e-3;            % [m]
N_piston = 4;   % 4 pistons per brake 
A_piston = N_piston * pi*(D_piston/2)^2;  % total piston area

mu_pad   = 0.45;                % pad friction coeff (guess)
lineP_max = 1e6;               % [Pa] ~10 bar, tune to taste

T_brake_max = 2 * mu_pad * lineP_max * A_piston * R_eff;  % factor 2: two pads
F_brake_wheel_max_rear = T_brake_max / wheelRadius;            % [N] at tyre

F_brake_wheel_max = F_brake_wheel_max_rear + F_brake_wheel_max_front;

%% Simulation loop with power-sensitive lap time scaling
for i = 1:length(xresMCP_laps)-1  % One less due to diff

    turnSign = TSignProfile(i);
    r_turn = RProfile(i);

    % --- velocity-dependent look-ahead size (keep your scaling) ---
    Nmin = 1;
    Nmax = 30;
    v_ref = speed_limit;
    alpha = max(0, min(velocity / v_ref, 1));   % 0..1
    Nlook = round(Nmin + (Nmax - Nmin)*alpha);

    % indices for look-ahead window
    Npts = length(RProfile);
    idxEnd = min(i + Nlook, Npts);

    % window of radii ahead (including current)
    R_window = RProfile(i:idxEnd);

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

%% BIKE SPECIFIC    
    % --- Roll angle from Cossalter 4.1.1 & 4.1.2 ---
%% END OF BIKE SPECIFIC

    % Lateral force demand
    if r_turn ~= Inf
        F_lat = M_effective * velocity^2 / r_turn;
    else
        F_lat = 0;
    end

    % Max total tire force (traction circle)
    Fz = (M_effective * g + F_drag); %+/- Fg
    F_tire_total = (tireFrictionCoeff * Fz);

    Fz_prev = Fz;

    if i == 1
        FTyreprofile = zeros(length(xresMCP_laps)-1,1);
    end

    FTyreprofile(i) = F_tire_total;

    % --- Cornering speed limit from mu and radius ---
    if isfinite(r_turn) && r_turn > 0
        v_corner_mu = sqrt(tireFrictionCoeff * g * R_min_forward);   % lateral friction-limited cornering speed
    else
        v_corner_mu = speed_limit;
    end

    % combine with your global limit
    v_limit = min(v_corner_mu, speed_limit);

    if i == 1
        VLIMprofile = zeros(length(xresMCP_laps)-1,1);
    end

    VLIMprofile(i) = v_limit;

    % --- Remaining longitudinal capacity from traction circle ---
    F_long_cap = sqrt(max(F_tire_total^2 - F_lat^2, 0));   % >=0

    % --- Power-limit in acceleration only ---
    if velocity > 0
        F_power_limit = Pmax_Tot / velocity;
    else
        F_power_limit = inf;   % at very low speed power limit isn't binding
    end

    % how far above the safe corner speed we are
    vdelta = max(0, velocity - v_limit);

    % choose how far above v_limit full braking should kick in
    brakeBandwidth = 7;   % [m/s], tune this value

    % braking intensity from 0→1
    brakeScale = min(1, vdelta / brakeBandwidth);

    % --- Simple "driver": decide whether to gas or brake ---
    % look at how far above / below the local speed limit we are

    if velocity < v_limit
        % ACCELERATION PHASE
        F_cmd = F_power_limit;          % try to use all available power
        F_cm = 0;
    elseif velocity > v_limit % + margin
        % BRAKING PHASE
        F_cm = (-F_brake_wheel_max * brakeScale);     % full brake request (negative)
        F_cmd = F_cm - F_drag - F_roll; %-Fg
    end

    % --- Apply traction-circle and power/brake limits with correct sign ---
    if F_cmd >= 0
        % accelerating: limited by traction & power
        F_long = min([F_cmd, F_long_cap, F_power_limit]);
    else
        % braking: negative, limited by traction & brake system
        F_long = F_cmd; %max(F_cmd, -F_long_max_brake);  % most negative allowed
    end

    if i == 1
        FCMDprofile = zeros(length(xresMCP_laps)-1,1);
    end

    FCMDprofile(i) = F_cmd;

    if i == 1
        FCMprofile = zeros(length(xresMCP_laps)-1,1);
    end

    FCMprofile(i) = F_cm;

    % Acceleration
    dv = (F_long - F_drag - F_roll) / M_effective;

    %  Only taper during acceleration, never during braking
    if dv > 0
    VtaperStart = 0.4 * speed_limit;   % start taper at 40% of Vmax

        if velocity > VtaperStart
        frac = (velocity - VtaperStart) / (speed_limit - VtaperStart); % 0..1
        frac = max(0, min(frac, 1));

        % Option A: linear taper (simple)
        taper = 1 - frac;

        % Option B: smooth taper (nicer) — uncomment if you want it
        % taper = 1 - (frac^2 * (3 - 2*frac));   % smoothstep

        dv = dv * taper;
        end
    end

    velocity = velocity + dv * dt_seg;

    % Clamp velocity at max achievable or speed limit
    velocity = max(0, min(velocity, speed_limit));  % clamp, no negative speeds

    ds = segmentLengths(i);                   % segment length
    dt_seg = ds / velocity;

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


drivecycle = [timeProfile, velocityProfile];
assignin('base', 'DC', drivecycle);

%% Plots

% Plot velocity over time
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

% ------------------------------------------------------

% Plot velocity as a gradient over the track layout
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
plot(timeProfile, RProfile(1:end-1))
legend('Instantaneous Radius')
xlabel('Time (s)')
ylabel('Meters (m)')
grid on
title('Radius Over Lap')


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
xlabel('Position')
ylabel('dt (s)')
grid on
title('delta Time Over Lap')

figure;
plot(timeProfile, accel)
xlabel('Position')
ylabel('dv (ms^-2)')
grid on
title('delta V Over Lap')

figure;
plot(timeProfile, FCMDprofile)
legend('Force')
xlabel('Time (s)')
ylabel('force (N)')
grid on
title('F_CMD Over Lap')
ylim([-10000 10000])

figure;
plot(timeProfile, FCMprofile)
legend('regen')
xlabel('Time (s)')
ylabel('force (N)')
grid on
title('FCM Over Lap')


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

%% ================= Velocity vs Distance =================
% segmentLengths is [Nseg x 1] where Nseg = length(xresMCP_laps)-1
% velocityProfile is also [Nseg x 1] (stored once per segment)

s_m = cumsum(segmentLengths(:));     % distance at END of each segment [m]
v_mps = velocityProfile(:);          % velocity at END of each segment [m/s]

% Safety check
N = min(length(s_m), length(v_mps));
s_m = s_m(1:N);
v_mps = v_mps(1:N);

% Optional: include start point (distance = 0)
s0_m  = [0; s_m];
v0_mps = [v_mps(1); v_mps];  % (or use your initial velocity 0.1 if you want)
% v0_mps = [0.1; v_mps];

% Package as a struct (nice for later use)
VelDist = struct();
VelDist.distance_m = s0_m;
VelDist.velocity_mps = v0_mps;

% % Save .mat
% save('VelocityVsDistance.mat', 'VelDist', 's_m', 'v_mps', 's0_m', 'v0_mps');
% 
% % Also push to base workspace if you want
% assignin('base','VelDist',VelDist);

%% Plot: Velocity vs Distance
figure;
plot(s0_m, v0_mps, 'LineWidth', 2);
grid on;
xlabel('Distance (m)', 'FontWeight','bold','FontSize',14);
ylabel('Velocity (m/s)', 'FontWeight','bold','FontSize',14);
title('Velocity vs Distance', 'FontWeight','bold','FontSize',16);
ax = gca; ax.FontWeight='bold'; ax.FontSize=14;


