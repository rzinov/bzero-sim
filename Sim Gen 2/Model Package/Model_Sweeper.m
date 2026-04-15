Upload_DataBase; % Runs Param Loader
modelName = 'TAYCAN_MODEL';
%% --- Parameter sets 

Motor_power_set = [1 2];
Max_power_motor_F_W_set   = [240e3 180e3];
Max_torque_motor_F_Nm_set = [340 255];
Max_power_motor_R_W_set   = [520e3 400e3];
Max_torque_motor_R_Nm_set = [900 675];


Shift_speed_set = [1 2 3 4];
shift_up_v_set   = [80  100 120 180];
shift_down_v_set = [60  80  100 120];

Torque_multiplication_set = [1 2 3 4 5];
Torque_multiplier_set     = [1 1.5 2 2.5 3];

% Base torque curves (store once; don't overwrite)
Max_torque_curve_F_set = [340;340;340;340;340;340;288.55;247.42;216.63;192.44;173.11;157.49;144.19;133.16;123.58]';
Max_torque_curve_R_set = [900.00;900.00;900.00;900.00;900.00;900.00;900.00;898.90;785.84;698.65;628.92;571.90;524.18;483.85;448.95]';

%% --- Run counts ---
nP = numel(Motor_power_set);
nS = numel(Shift_speed_set);
nQ = numel(Torque_multiplication_set);

nRuns = nP*nS*nQ;
disp("Total runs = " + nRuns);

%% --- Logs ---
ParamLog = table('Size',[nRuns 12], ...
    'VariableTypes',{'double','double','double','double','double','double','double', 'double', 'double', 'double', 'double', 'double'}, ...
    'VariableNames',{'P_max_F_W','T_max_F_Nm','P_max_R_W','T_max_R_Nm','ShiftUp','ShiftDown','Torque_x', 'Ipack', 'Np', 'Ns', 'batt_mass', 'Tot_vehicle_mass'});

ResultSet = cell(nRuns,1);

runIdx = 0;

%% --- (Optional) speed-up ---
load_system("DO_NOT_TOUCHY_V2.slx");
%set_param("DO_NOT_TOUCHY_V2.slx",'FastRestart','on');  

for idp = 1:nP
    % --- Motor power/torque set ---
    P_max_F = Max_power_motor_F_W_set(idp);
    T_max_F = Max_torque_motor_F_Nm_set(idp);
    P_max_R = Max_power_motor_R_W_set(idp);
    T_max_R = Max_torque_motor_R_Nm_set(idp);

        for ids = 1:nS
            % --- Shift threshold set ---
            shift_up_v   = shift_up_v_set(ids);
            shift_down_v = shift_down_v_set(ids);

            for idq = 1:nQ
                % --- Torque multiplication set ---
                Torque_x = Torque_multiplier_set(idq);

                % --- Updated model workspace variables ---
                Max_power_motor_F_W   = P_max_F * Torque_x;
                Max_torque_motor_F_Nm = T_max_F * Torque_x;      % optional: scale scalar too
                Max_power_motor_R_W   = P_max_R * Torque_x;
                Max_torque_motor_R_Nm = T_max_R * Torque_x;      % optional: scale scalar too
                Ipack = (Max_power_motor_R_W + Max_power_motor_F_W) / (Ns * V_init);
                Np = ceil(Ipack / Imax);
                batt_mass = (Np * Ns * cell_mass_g)/1000;
                Tot_vehicle_mass_kg = Vehicle_mass_unladen_kg + Driver_mass_kg + batt_mass;

                % Scale the whole torque curves (RPM unchanged)
                Max_torque_curve_F_Nm = Torque_x * Max_torque_curve_F_set;
                Max_torque_curve_R_Nm = Torque_x * Max_torque_curve_R_set;

                % --- Log params ---
                runIdx = runIdx + 1;
                ParamLog(runIdx,:) = {Max_power_motor_F_W, Max_torque_motor_F_Nm, Max_power_motor_R_W, Max_torque_motor_R_Nm, shift_up_v, shift_down_v, Torque_x, Ipack, Np, Ns, batt_mass, Tot_vehicle_mass_kg};
                disp('RUN NUMBER:')
                disp(runIdx)
                disp(ParamLog(runIdx,:))
                % --- Run ---
                 
                try
                    out = sim("DO_NOT_TOUCHY_V2.slx");
                    disp('ENDSOC =')
                    disp(out.SOC.Data(end))
                    ResultSet{runIdx} = out;
                catch ME
                    ResultSet{runIdx} = [];
                    warning("Run %d/%d failed: %s", runIdx, nRuns, ME.message);
                end
            end
        end
    
end
%set_param(modelName,'FastRestart','off');  % optional
save('SweepResults.mat','ParamLog','ResultSet');
disp("Done. Saved SweepResults.mat");
