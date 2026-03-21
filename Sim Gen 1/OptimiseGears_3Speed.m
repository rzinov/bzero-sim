% OptimizeGears_3Speed.m
clear; clc; close all;

% --- GEAR LIMITS & INITIAL GUESS ---
gears_initial = [2.5, 1.8, 1.0]; 
lb = [1.5, 1.0, 0.5];   % Lower bounds [Gear1, Gear2, Gear3]
ub = [4.0, 2.5, 1.5];   % Upper bounds [Gear1, Gear2, Gear3]

% Constraints: Gear1 > Gear2 > Gear3 (by at least 0.2 gaps)
A = [-1,  1,  0;   
      0, -1,  1];  
b = [-0.2; -0.2]; 

options = optimoptions('patternsearch', 'Display', 'iter', 'UseParallel', false);
sprockets = [14, 45]; 

% --- 1. INITIAL GUESS ---
fprintf('INITIAL 3-SPEED GUESS\n');
[lap, energy, top_speed] = Sim(gears_initial, false, sprockets, true); 
fprintf('Initial -> Lap: %.2fs | Top Speed: %.1f km/h\n', lap, top_speed);

fprintf('\nStarting 3-Speed Gear Optimisation\n');
optimal_gears = patternsearch(@costFunction, gears_initial, A, b, [], [], lb, ub, [], options);

% --- 2. OPTIMISED WINNER ---
fprintf('\nOPTIMAL 3-SPEED GEARS FOUND\n');
fprintf('1st Gear: %.3f\n2nd Gear: %.3f\n3rd Gear: %.3f\n', optimal_gears(1), optimal_gears(2), optimal_gears(3));

fprintf('\nEVALUATING FINAL OPTIMIZED GEARS\n');
[opt_lap, opt_energy, opt_topSpeed] = Sim(optimal_gears, false, sprockets, true);
fprintf('Optimised -> Lap: %.2fs | Energy: %.2f kWh | Top Speed: %.1f km/h\n', opt_lap, opt_energy / 3.6e6, opt_topSpeed);

% --- 3. FEASIBILITY MESH PLOT (Holding 2nd Gear Constant) ---
g1_range = linspace(max(lb(1), optimal_gears(1)-0.5), min(ub(1), optimal_gears(1)+0.5), 6);
g3_range = linspace(max(lb(3), optimal_gears(3)-0.3), min(ub(3), optimal_gears(3)+0.3), 6);
opt_g2 = optimal_gears(2); % Hold 2nd gear steady

[G1, G3] = meshgrid(g1_range, g3_range);
LapMesh = zeros(size(G1));

for i = 1:size(G1, 1)
    for j = 1:size(G1, 2)
        % Ensure constraints still hold with the static 2nd gear
        if (G1(i,j) > opt_g2 + 0.1) && (opt_g2 > G3(i,j) + 0.1)
            [l_time, ~, ~] = Sim([G1(i,j), opt_g2, G3(i,j)], false, sprockets, false);
            LapMesh(i,j) = l_time;
        else
            LapMesh(i,j) = NaN;
        end
    end
end

figure;
surf(G1, G3, LapMesh);
colorbar; colormap(turbo);
xlabel('1st Gear Ratio', 'FontWeight', 'bold');
ylabel('3rd Gear Ratio', 'FontWeight', 'bold');
zlabel('Lap Time (s)', 'FontWeight', 'bold');
title(sprintf('3-Speed: 1st vs 3rd Gear (2nd Gear = %.2f)', opt_g2), 'FontWeight', 'bold');
set(gca, 'View', [-45, 45]);

% =========================================================================
function cost = costFunction(gears)
    try
        [lapTime, energy_J, topSpeed_kmh] = Sim(gears, false, [14, 45], false); 
        energy_kWh = energy_J / 3.6e6;
        
        weight_time = 1.0;     
        weight_energy = 20.0;  
        
        speed_penalty = 0;
        if topSpeed_kmh < 280
            speed_penalty = (280 - topSpeed_kmh) * 2.5; 
        end
        
        cost = (lapTime * weight_time) + (energy_kWh * weight_energy) + speed_penalty;
        if isnan(lapTime) || lapTime > 300, cost = 1e6; end
    catch
        cost = 1e6; 
    end
end