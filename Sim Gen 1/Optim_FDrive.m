clear; clc;

test_ratios = 2.0 : 0.5 : 8.0; % Sweeping from ratio 2.0 to 8.0
num_tests = length(test_ratios);
lap_times = zeros(num_tests, 1);
energies_kWh = zeros(num_tests, 1);

for i = 1:num_tests
    sprockets = [1, test_ratios(i)];
    
    % Call Sim: empty gearRatios [], DirectDrive = true, pass sprockets, isDebug = false
    [lap, energy] = Sim([], true, sprockets, false); 
    
    lap_times(i) = lap;
    energies_kWh(i) = energy / 3.6e6;
    fprintf('Ratio: %.1f | Lap: %.2fs | Energy: %.2f kWh\n', test_ratios(i), lap, energies_kWh(i));
end

% --- PLOTTING ---
figure;
yyaxis left;
plot(test_ratios, lap_times, '-bo', 'LineWidth', 2);
ylabel('Lap Time (s)');
yyaxis right;
plot(test_ratios, energies_kWh, '-rx', 'LineWidth', 2);
ylabel('Energy Consumed (kWh)');
xlabel('Drive Ratio');
title('Direct Drive Ratio Optimization');
grid on;