% RunOptimizer.m
clear; clc;

% 1. Define the starting point (Your best guess)
initial_gears = [2.20, 1.60, 1.10, 0.76];

% 2. Define the Cost Function
% We create a "Wrapper" because fminsearch needs a function that returns a SINGLE number (Cost)
% We add penalties so it doesn't choose impossible gears (like 1st gear being taller than 2nd)
CostFunc = @(g) ObjectiveFunction(g);

% 3. Run Optimization
options = optimset('Display', 'iter', 'TolX', 1e-3);
fprintf('Optimizing Gears for THIS SPECIFIC TRACK...\n');
optimal_gears = fminsearch(CostFunc, initial_gears, options);

% 4. Show Results
fprintf('--------------------------------------\n');
fprintf('OPTIMIZED GEARS FOR TRACK:\n');
fprintf('Gear 1: %.3f\n', optimal_gears(1));
fprintf('Gear 2: %.3f\n', optimal_gears(2));
fprintf('Gear 3: %.3f\n', optimal_gears(3));
fprintf('Gear 4: %.3f\n', optimal_gears(4));

% 5. Run one last time to plot the result
Sim(optimal_gears); 


% ---------------------------------------------------------
% Helper Function: Calculates "Cost" (Lap Time + Penalties)
% ---------------------------------------------------------
function cost = ObjectiveFunction(gears)
    % Constraint 1: Order (G1 > G2 > G3 > G4)
    if gears(1) <= gears(2) || gears(2) <= gears(3) || gears(3) <= gears(4)
        cost = 10000; return; 
    end
    
    % Constraint 2: Top Gear must be usable (not < 0.5)
    if gears(4) < 0.5
        cost = 10000; return;
    end

    try
        % RUN YOUR SIMULATION
        % We only care about the first output (Lap Time)
        lap_time = Sim(gears);
        
        % The cost is simply the lap time
        cost = lap_time;
        
        % Optional: Penalty if top speed < 290 (Force it to prioritize top speed)
        % max_speed = ... (would need to extract from Sim)
        
    catch
        % If Sim crashes due to bad gears, return high cost
        cost = 10000;
    end
end