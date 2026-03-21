function [trackDataOut] = processTrack(filename)
    % processTrack - 5-Line Surface Version (Fixed Output Compatibility)
    
    %% 1. Import Data
    raw = readmatrix(filename, 'NumHeaderLines', 1);
    name = 'Aragon';
    
    % --- COLUMN MAPPING ---
    % Col 2-4:   Outer Left
    % Col 5-7:   Inner Left
    % Col 8-10:  Center
    % Col 11-13: Inner Right
    % Col 14-16: Outer Right
    
    x_L_out = raw(:,2);   y_L_out = raw(:,3);
    x_L_in  = raw(:,5);   y_L_in  = raw(:,6);
    x_C     = raw(:,8);   y_C     = raw(:,9);
    x_R_in  = raw(:,11);  y_R_in  = raw(:,12);
    x_R_out = raw(:,14);  y_R_out = raw(:,15);
    
    % Z coordinates (Elevation Profiles)
    z_L_out = raw(:,4);
    z_L_in  = raw(:,7);
    z_C     = raw(:,10);
    z_R_in  = raw(:,13);
    z_R_out = raw(:,16);
    
    %% 1.2 Force Loop Closure
    gap_dist = hypot(x_C(1) - x_C(end), y_C(1) - y_C(end));
    if gap_dist > 0.1 
        % Append first point to end for all arrays
        x_L_out(end+1)=x_L_out(1); y_L_out(end+1)=y_L_out(1); z_L_out(end+1)=z_L_out(1);
        x_L_in(end+1) =x_L_in(1);  y_L_in(end+1) =y_L_in(1);  z_L_in(end+1) =z_L_in(1);
        x_C(end+1)    =x_C(1);     y_C(end+1)    =y_C(1);     z_C(end+1)    =z_C(1);
        x_R_in(end+1) =x_R_in(1);  y_R_in(end+1) =y_R_in(1);  z_R_in(end+1) =z_R_in(1);
        x_R_out(end+1)=x_R_out(1); y_R_out(end+1)=y_R_out(1); z_R_out(end+1)=z_R_out(1);
    end

    %% 2. Pre-processing & Interpolation
    centerXY = [x_C y_C];
    stepLengths = sqrt(sum(diff(centerXY,[],1).^2,2));
    stepLengths = [0; stepLengths]; 
    cumulativeLen = cumsum(stepLengths); 
    
    % Create High-Res Grid (1500 points)
    nseg = 1500;
    finalStepLocs = linspace(0, cumulativeLen(end), nseg);
    
    % Helper function to interpolate a vector 'v'
    interp_track = @(v) interp1(cumulativeLen, v, finalStepLocs, 'pchip')';
    
    % Interpolate ALL X, Y, and Z arrays
    xl_out = interp_track(x_L_out);  yl_out = interp_track(y_L_out);
    xl_in  = interp_track(x_L_in);   yl_in  = interp_track(y_L_in);
    xc     = interp_track(x_C);      yc     = interp_track(y_C);
    xr_in  = interp_track(x_R_in);   yr_in  = interp_track(y_R_in);
    xr_out = interp_track(x_R_out);  yr_out = interp_track(y_R_out);
    
    % Z Planes (Elevation)
    zl_out = interp_track(z_L_out);
    zl_in  = interp_track(z_L_in);
    zc     = interp_track(z_C);
    zr_in  = interp_track(z_R_in);
    zr_out = interp_track(z_R_out);
    
    %% 3. Solver Setup (3D "Best Fit Plane")
    % Instead of just connecting the outer edges, we calculate a best fit line through 5 points to account for the track crown/dip.
    
    % X/Y Boundaries (Fixed geometric walls - unchanged)
    xin  = xr_out; 
    yin  = yr_out;
    xout = xl_out;
    yout = yl_out;
    
    delx = xout - xin;
    dely = yout - yin;
    
    % --- CALCULATE BEST FIT Z-PLANE ---
    n = length(xin);
    zin_fit  = zeros(n,1); % Intercept
    delz_fit = zeros(n,1); % Change across width
    
    % Relative positions of the 5 data columns (0 = Right Wall, 1 = Left Wall)
    % [RightOut, RightIn, Center, LeftIn, LeftOut]
    
    for i = 1:n
        % --- 1. Calculate the Reference Width (Right Outer to Left Outer) ---
        % This is the full width of the track slice
        total_w = hypot(xl_out(i) - xr_out(i), yl_out(i) - yr_out(i));
    
        % Safety check to avoid dividing by zero
        if total_w < 1e-6
            total_w = 1.0; 
        end

        % --- 2. Calculate Relative Distances (0 to 1) ---
    
        % d1: Right Outer (Reference point, always 0)
        d1 = 0;
    
        % d2: Right Inner (Distance from Right Outer)
        dist_r_in = hypot(xr_in(i) - xr_out(i), yr_in(i) - yr_out(i));
        d2 = dist_r_in / total_w;
    
        % d3: Center Line
        dist_c = hypot(xc(i) - xr_out(i), yc(i) - yr_out(i));
        d3 = dist_c / total_w;
    
        % d4: Left Inner
        dist_l_in = hypot(xl_in(i) - xr_out(i), yl_in(i) - yr_out(i));
        d4 = dist_l_in / total_w;
    
        % d5: Left Outer (Always 1.0 by definition)
        d5 = 1.0;

        % Collect the "real" transverse locations
        a_vals = [d1, d2, d3, d4, d5];
    
        % --- 3. Fit the Line ---
        % Get the Z-heights for this slice
        z_slice = [zr_out(i), zr_in(i), zc(i), zl_in(i), zl_out(i)];
    
        % Polyfit order 1
        p = polyfit(a_vals, z_slice, 1);
    
        delz_fit(i) = p(1); % Slope (Total height difference)
        zin_fit(i)  = p(2); % Intercept (Height at Right Outer edge)
    end
    
    % Update the solver variables
    delz = delz_fit;
    zin  = zin_fit;
    
    %% Matrix Definition & MCP Solver
    H = zeros(n);
    B = zeros(size(delx)).';
    
    for i=2:n-1
        
        % i-1 Interaction
        H(i-1,i-1) = H(i-1,i-1) + delx(i-1)^2         + dely(i-1)^2         + delz(i-1)^2;
        H(i-1,i)   = H(i-1,i)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i) - 2*delz(i-1)*delz(i);
        H(i-1,i+1) = H(i-1,i+1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1) + delz(i-1)*delz(i+1);
        
        % i Interaction (Main Diagonal)
        H(i,i-1)   = H(i,i-1)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i) - 2*delz(i-1)*delz(i);
        H(i,i)     = H(i,i )    + 4*delx(i)^2         + 4*dely(i)^2         + 4*delz(i)^2;
        H(i,i+1)   = H(i,i+1)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1) - 2*delz(i)*delz(i+1);
        
        % i+1 Interaction
        H(i+1,i-1) = H(i+1,i-1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1) + delz(i-1)*delz(i+1);
        H(i+1,i)   = H(i+1,i)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1) - 2*delz(i)*delz(i+1);
        H(i+1,i+1) = H(i+1,i+1) + delx(i+1)^2         + dely(i+1)^2         + delz(i+1)^2;
        
        % Geometric Vector B (With Z Curvature)
        termX = (xin(i+1)+xin(i-1)-2*xin(i));
        termY = (yin(i+1)+yin(i-1)-2*yin(i));
        termZ = (zin(i+1)+zin(i-1)-2*zin(i)); % Uses the fitted zin
        
        B(1,i-1) = B(1,i-1) + 2*termX*delx(i-1) + 2*termY*dely(i-1) + 2*termZ*delz(i-1);
        B(1,i)   = B(1,i)   - 4*termX*delx(i)   - 4*termY*dely(i)   - 4*termZ*delz(i);
        B(1,i+1) = B(1,i+1) + 2*termX*delx(i+1) + 2*termY*dely(i+1) + 2*termZ*delz(i+1);
    end
    
    lb = zeros(n,1); ub = ones(size(lb));
    Aeq = zeros(1,n); Aeq(1)=1; Aeq(end)=-1; beq=0;
    
    options = optimoptions('quadprog','Display','off');
    [resMCP,~,~,~] = quadprog(2*H,B',[],[],Aeq,beq,lb,ub,[],options);
    
    % Calculate Final Path (Using geometric boundaries for X/Y, but fitted for Z)
    x_opt = xin + resMCP.*delx;
    y_opt = yin + resMCP.*dely;
    z_opt = zin + resMCP.*delz;
    
    %% 4. Advanced Z & Banking Calculation
    banking_angle = zeros(n,1);
    
    for i = 1:n
        width_i = hypot(xout(i)-xin(i), yout(i)-yin(i));
        height_diff = delz(i);
        z_ratio = height_diff / width_i;
        z_ratio = max(-1,min(1,z_ratio));
        if width_i > 1e-6
            banking_angle(i) = atan(z_ratio);
        else
            banking_angle(i) = 0;
        end

    end

        %% 5. Corner Radius Solver
    RProfileLap  = zeros(n,1);
    TSignLap     = zeros(n,1);
    
    for i = 1:n
        if i == 1 
            u1=n-1; 
            u2=i; 
            u3=i+1;
        elseif i==n 
            u1=i-1; 
            u2=i; 
            u3=2;
        else 
            u1=i-1; 
            u2=i; 
            u3=i+1; 
        end 
        
        x1=x_opt(u1); y1=y_opt(u1); 
        x2=x_opt(u2); y2=y_opt(u2); 
        x3=x_opt(u3); y3=y_opt(u3);
        
        a_len = hypot(x2-x3, y2-y3); 
        b_len = hypot(x1-x3, y1-y3); 
        c_len = hypot(x1-x2, y1-y2);
        area2 = (x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2));
        A = 0.5*abs(area2);
        
        if A > 0 
            R_mag = (a_len*b_len*c_len)/(4*A); 
            turnSign = sign(area2); 
        else 
            R_mag = inf; turnSign = 0; 
        end
        RProfileLap(i) = R_mag; 
        TSignLap(i) = turnSign; 
    end


    %% 6. Drive Cycle Generator Path
    nLaps = 1;
    % Create the _laps variables
    xresMCP_laps = repmat(x_opt, nLaps, 1);
    yresMCP_laps = repmat(y_opt, nLaps, 1);
    TSignProfile = repmat(TSignLap,     nLaps, 1);
    
    % Compute current lap length & Scale
    dx_seg = diff(xresMCP_laps); dy_seg = diff(yresMCP_laps);
    segmentLengths = sqrt(dx_seg.^2 + dy_seg.^2);
    lapLength = sum(segmentLengths);
    targetLap = 5078 * nLaps; 
    scale_factor = targetLap / lapLength;
    % Apply scaling
    xresMCP_laps = xresMCP_laps * scale_factor;
    yresMCP_laps = yresMCP_laps * scale_factor;
    xin     = xin  * scale_factor;
    yin     = yin  * scale_factor;
    xout    = xout * scale_factor;
    yout    = yout * scale_factor;
    xc      = xc   * scale_factor;
    yc      = yc   * scale_factor;
    x_opt = x_opt *scale_factor;
    y_opt   = y_opt * scale_factor;
    z_opt   = z_opt * scale_factor;
    RProfile = repmat(RProfileLap, nLaps, 1) * scale_factor;
    % Recompute lengths
    dx_seg = diff(xresMCP_laps); dy_seg = diff(yresMCP_laps);
    segmentLengths = sqrt(dx_seg.^2 + dy_seg.^2);

    
    
    %% Output Packaging
    trackDataOut.xresMCP_laps = xresMCP_laps;
    trackDataOut.yresMCP_laps = yresMCP_laps;
    trackDataOut.RProfile = RProfile;
    trackDataOut.TSignProfile = TSignProfile;
    trackDataOut.segmentLengths = segmentLengths;
    trackDataOut.zt = z_opt;      
    trackDataOut.xt = xc;         
    trackDataOut.yt = yc;        
    trackDataOut.xin = xin; trackDataOut.xout = xout;
    trackDataOut.yin = yin; trackDataOut.yout = yout;
    trackDataOut.xresMCP = x_opt; 
    trackDataOut.yresMCP = y_opt;
    trackDataOut.finalStepLocs = finalStepLocs;
    trackDataOut.scale_factor = scale_factor;
    trackDataOut.name = name;
    trackDataOut.banking = banking_angle;
end