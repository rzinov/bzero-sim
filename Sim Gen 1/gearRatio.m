% OptimizeGears.m
clear; clc; close all;

gears_initial = [2.5, 1.8, 1.2, 0.77];
lb = [1.8, 1.3, 1.0, 0.75];
ub = [3.0, 2.2, 1.5, 0.95];

% Constraints: Gear 1 > Gear 2 > Gear 3 > Gear 4
A = [-1,  1,  0,  0;   
      0, -1,  1,  0;   
      0,  0, -1,  1];  
b = [-0.1; -0.1; -0.1]; 

options = optimoptions('patternsearch', 'Display', 'iter', 'UseParallel', false);

% --- 1. SHOW THE INITIAL GUESS ---
fprintf('--- EVALUATING INITIAL GUESS ---\n');
Sim(gears_initial, true); 

fprintf('\nStarting Gear Optimization (Running silently in background)...\n');

% Run the optimizer
optimal_gears = patternsearch(@costFunction, gears_initial, A, b, [], [], lb, ub, [], options);

% --- 2. SHOW THE OPTIMIZED WINNER ---
fprintf('\n--- OPTIMAL GEARS FOUND ---\n');
fprintf('Gear 1: %.3f\nGear 2: %.3f\nGear 3: %.3f\nGear 4: %.3f\n', optimal_gears);

fprintf('\n--- EVALUATING FINAL OPTIMIZED GEARS ---\n');
Sim(optimal_gears, false, nil, true); % Plot the final winner!

% =========================================================================
function cost = costFunction(gears)
    try
        % RUN SILENTLY: Notice the 'false' flag here!
        [lapTime, energy_J] = Sim(gears, false); 
        
        energy_kWh = energy_J / 3.6e6;
        
        weight_time = 1.0;     
        weight_energy = 50.0;  % 0 = Pure Qualifying Mode
        
        cost = (lapTime * weight_time) + (energy_kWh * weight_energy);
        
        if isnan(lapTime) || lapTime > 300
            cost = 1e6;
        end
    catch
        cost = 1e6; 
    end
end