% Optim_FDrive.m
clear; clc;

test_ratios = 2.0 : 0.1 : 8.0; % Sweeping from ratio 2.0 to 8.0
num_tests = length(test_ratios);

% Preallocate arrays
lap_times = zeros(num_tests, 1);
energies_kWh = zeros(num_tests, 1);
top_speeds = zeros(num_tests, 1);

for i = 1:num_tests
    sprockets = [1, test_ratios(i)];
    
    % Call Sim
    [lap, energy, top_speed] = BZEROV4_RELEASE([], true, sprockets, false); 
    
    lap_times(i) = lap;
    energies_kWh(i) = energy / 3.6e6;
    top_speeds(i) = top_speed;
    
    fprintf('Ratio: %.1f | Lap: %.2fs | Energy: %.2f kWh | Top Speed: %.1f km/h\n', ...
        test_ratios(i), lap, energies_kWh(i), top_speed);
end

% --- PLOTTING ---
figure;

% Lap Time vs Energy
subplot(2,1,1);
yyaxis left;
plot(test_ratios, lap_times, '-bo', 'LineWidth', 2);
ylabel('Lap Time (s)', 'FontWeight', 'bold');
yyaxis right;
plot(test_ratios, energies_kWh, '-rx', 'LineWidth', 2);
ylabel('Energy (kWh)', 'FontWeight', 'bold');
title('Direct Drive: Lap Time & Energy', 'FontWeight', 'bold');
grid on;

% Top Speed
subplot(2,1,2);
plot(test_ratios, top_speeds, '-go', 'LineWidth', 2);
ylabel('Top Speed (km/h)', 'FontWeight', 'bold');
xlabel('Drive Ratio', 'FontWeight', 'bold');
title('Direct Drive: Max Speed', 'FontWeight', 'bold');
grid on;