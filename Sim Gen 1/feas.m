% Feasibility_Report_Graphs.m
clear; clc; close all;

%% 1. Input Data
setup_names = {'Direct Drive (Speed)', 'Direct Drive (Lap)', '2-Speed (Opt)', '3-Speed (Opt)'};

% Data arrays
lap_times  = [140.03, 136.11, 131.10, 130.17]; % Seconds (Lower is better)
top_speeds = [260.52, 207.76, 260.00, 259.28]; % km/h (Higher is better)
energies   = [1.11,   1.06,   1.13,   1.14];   % kWh (Lower is better)

% Custom colors for the setups (Deep Blue, Light Blue, Orange, Red)
colors = [0 0.4470 0.7410;  
          0.3010 0.7450 0.9330; 
          0.8500 0.3250 0.0980; 
          0.6350 0.0780 0.1840];

%% 2. Figure 1: The Core Metrics Dashboard
figure('Position', [100, 100, 1000, 600], 'Name', 'Drivetrain Feasibility Dashboard');

% Subplot 1: Lap Time
subplot(1, 3, 1);
b1 = bar(lap_times, 'FaceColor', 'flat');
b1.CData = colors;
ylabel('Lap Time (Seconds)', 'FontWeight', 'bold');
set(gca, 'XTickLabel', setup_names, 'XTickLabelRotation', 45);
title('Lap Time Comparison', 'FontWeight', 'bold');
grid on;
% Add data labels on top of bars
for i = 1:numel(lap_times)
    text(i, lap_times(i) + 1, sprintf('%.1fs', lap_times(i)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
ylim([125 145]); % Zoom in to highlight differences

% Subplot 2: Top Speed
subplot(1, 3, 2);
b2 = bar(top_speeds, 'FaceColor', 'flat');
b2.CData = colors;
ylabel('Top Speed (km/h)', 'FontWeight', 'bold');
set(gca, 'XTickLabel', setup_names, 'XTickLabelRotation', 45);
title('Top Speed Achieved', 'FontWeight', 'bold');
grid on;
% Add data labels
for i = 1:numel(top_speeds)
    text(i, top_speeds(i) + 5, sprintf('%.0f', top_speeds(i)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
ylim([190 280]);
yline(280, 'r--', 'Target Speed (280)', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');

% Subplot 3: Energy Consumption
subplot(1, 3, 3);
b3 = bar(energies, 'FaceColor', 'flat');
b3.CData = colors;
ylabel('Energy Consumed (kWh)', 'FontWeight', 'bold');
set(gca, 'XTickLabel', setup_names, 'XTickLabelRotation', 45);
title('Energy Consumption', 'FontWeight', 'bold');
grid on;
for i = 1:numel(energies)
    text(i, energies(i) + 0.02, sprintf('%.2f', energies(i)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
ylim([1.0 1.2]);

%% 3. Figure 2: The Trade-Off Bubble Chart
figure('Position', [150, 150, 800, 600], 'Name', 'Pareto Trade-Off Analysis');

hold on;
% Scale bubble sizes based on top speed for visual pop (e.g., square it and scale)
bubble_sizes = (top_speeds / max(top_speeds)).^4 * 1500; 

for i = 1:numel(setup_names)
    scatter(energies(i), lap_times(i), bubble_sizes(i), colors(i,:), 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', setup_names{i});
    
    % Add text labels slightly offset from the bubbles
    text(energies(i) + 0.002, lap_times(i) + 0.5, setup_names{i}, ...
        'FontWeight', 'bold', 'FontSize', 10);
end

xlabel('Energy Consumed (kWh) -> Lower is Better', 'FontWeight', 'bold', 'FontSize', 12);
ylabel('Lap Time (Seconds) -> Lower is Better', 'FontWeight', 'bold', 'FontSize', 12);
title('Drivetrain Trade-off: Lap Time vs Energy (Bubble Size = Top Speed)', 'FontWeight', 'bold', 'FontSize', 14);
grid on;
legend('Location', 'northeast');


hold off;