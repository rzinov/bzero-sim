whlrpm = velocityProfile * (1/(0.6/2));
motorrpm = whlrpm * 3.68 * 30/pi;


PPeak_kW = [0, 3.142, 6.283, 9.425, 12.566, 15.708, 18.85, 21.991, 25.133, 28.274, 31.416, ...
    34.558, 37.699, 40.841, 43.982, 47.124, 47.8, 48];
RPM_peakPower = [0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, ...
    3250, 3500, 3750, 4000, 7500];

motorRPM = interp1(RPM_peakPower, PPeak_kW, motorrpm, 'linear', 'extrap');

figure;
plot(timeProfile, motorRPM)
grid on
