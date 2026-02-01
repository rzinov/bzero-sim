function [trackDataOut] = processTrack(filename)
    %Acts as module to allow the code to take any track data just by calling a function
    %% Processing track data
    track = readmatrix(filename);
    name = 'Aragon';
    
    % track data - first point repeated
    data = track;

    % x,y,z and track width data
    y =  data(:,1);
    x =  data(:,2);
    twr = data(:,3);
    twl = data(:,4);
    z = data(:,5);
    
    % smooths elevation to create a coherent gradient
    z_smooth = smoothdata(z, 'gaussian', 25);
    
    % higher no. of segments causes trajectory to follow the reference line
    nseg = 1500;

    % interpolate data to get finer curve with equal distances between each segment
    pathXY = [x y];

    stepLengths = sqrt(sum(diff(pathXY,[],1).^2,2)); % - prepares for interpolation, pythagorous theorem for even distances
    stepLengths = [0; stepLengths]; % - starting point
    cumulativeLen = cumsum(stepLengths); % - model length of track
    finalStepLocs = linspace(0,cumulativeLen(end), nseg);
    finalPathXY = interp1(cumulativeLen, pathXY, finalStepLocs);
    
    % track centerline
    xt = finalPathXY(:,1);
    yt = finalPathXY(:,2);
    zt = interp1(cumulativeLen, z_smooth, finalStepLocs, 'pchip')';

    % track widths
    twrt = interp1(cumulativeLen, twr, finalStepLocs,'spline')';
    twlt = interp1(cumulativeLen, twl, finalStepLocs,'spline')';
    
    % normal direction for each vertex
    dx = gradient(xt);
    dy = gradient(yt);
    dL = hypot(dx,dy);
    
    % uses the track width (6m) data to turn them into actual coordinates
    xoff = @(a) -a.*dy./dL + xt;
    yoff = @(a)  a.*dx./dL + yt;
    
    % offset data
    offset = [-twrt twlt];
    xin  = xoff(offset(:,1));      
    yin  = yoff(offset(:,1));
    xout = xoff(offset(:,2));      
    yout = yoff(offset(:,2));
    
    % Form delta matrices
    delx = xout - xin;
    dely = yout - yin;
    
    % Store for plotting later
    trackDataRaw = [xt yt xin yin xout yout];

    %% Matrix Definition & MCP Solver
    % GOAL: Find the smoothest path by minimizing curvature (Acceleration^2)
    % Solves  for 'a' (resMCP) which is the position of the bike along the track width
    % a  = 0 is innermost wall, a = 1 is outermost wall
    % x(i) = Innermost + a*width of track
    n = numel(delx);
    H = zeros(n);
    B = zeros(size(delx)).';
    
    %% Builds H Matrix (cost matrix)
    % Matrix defines the cost of the path's shape
    % Expands the term (x(i-1) - 2x(i) + x(i+1))^2.
    % - Diagonal terms (X X X): The "squared terms", penalty for sharp angles.
    % - Off-diagonal terms (X X 0): The "cross terms" (2AB), reward for moving in line with neighbors (smoothing).
    for i=2:n-1
        % Previous neighbour interaction (i-1)
        H(i-1,i-1) = H(i-1,i-1) + delx(i-1)^2         + dely(i-1)^2;
        H(i-1,i)   = H(i-1,i)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
        H(i-1,i+1) = H(i-1,i+1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);
        
        % Current point interaction (i) - Main penalty
        H(i,i-1)   = H(i,i-1)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
        H(i,i)     = H(i,i )    + 4*delx(i)^2         + 4*dely(i)^2;
        H(i,i+1)   = H(i,i+1)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);
        
        % Next neighbour interaction (i+1)
        H(i+1,i-1) = H(i+1,i-1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);
        H(i+1,i)   = H(i+1,i)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);
        H(i+1,i+1) = H(i+1,i+1) + delx(i+1)^2         + dely(i+1)^2;
    end
    
    %% Builds B Matrix (geometric reference)
    % Encodes the curvature of the inner wall of the track
    % B = (Curvature of inner wall) * (Direction of track width)
    % Acts as a geometric constraint the solver has to work with
    for i=2:n-1
        B(1,i-1) = B(1,i-1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i-1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i-1);
        B(1,i)   = B(1,i)   - 4*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i)   - 4*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i);
        B(1,i+1) = B(1,i+1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i+1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i+1);
    end
    
    % Constrants: 'a' must be between 0 and 1
    lb = zeros(n,1);
    ub = ones(size(lb));

    % Forces start and end points to match
    Aeq = zeros(1,n); Aeq(1)=1; Aeq(end)=-1; beq=0;
    
    %% Solver
    % Solves the linear system H * a + B = 0
    % Finds values of a (resMCP) that minimise cost close to 0
    options = optimoptions('quadprog','Display','off');
    [resMCP,~,~,~] = quadprog(2*H,B',[],[],Aeq,beq,lb,ub,[],options);
    
    % Calculates final coordinates : Inner wall + a * Width_vector
    xresMCP = xin + resMCP.*delx;
    yresMCP = yin + resMCP.*dely;
    
    %% Corner Radius Solver
    % calculates instantaneous radius of curvature for every point on the path
    % forms a triangle using points (previous, current, next) and finds radius of the circle that passes through all 3 vertices
    nLap = length(xresMCP);
    RProfileLap  = zeros(nLap,1);
    TSignLap     = zeros(nLap,1);
    ARProfileLap = zeros(nLap,1);
    
    for i = 1:nLap
        % identifies the local triangle
        % handles edge cases for start and end of array
        if i == 1 
            u1=1; u2=2; u3=3; 
        elseif i==nLap
            u1=nLap-2; u2=nLap-1; u3=nLap; 
        else
            u1=i-1; u2=i; u3=i+1; 
        end 
        
        % extracts coordinates
        x1=xresMCP(u1); y1=yresMCP(u1); 
        x2=xresMCP(u2); y2=yresMCP(u2); 
        x3=xresMCP(u3); y3=yresMCP(u3);
        
        % calculates side lengths 
        a = hypot(x2-x3, y2-y3); 
        b = hypot(x1-x3, y1-y3); 
        c = hypot(x1-x2, y1-y2);
        
        % calculates triangle area, area2 is signed (positive is counter clockwise - left turn, negative is clockwise - right turn)
        area2 = (x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2));
        A = 0.5*abs(area2);
        
        % circumradius solver
        if A > 0 
            R_mag = (a*b*c)/(4*A); 
            turnSign = sign(area2); % + Left, - Right
        else 
            % Colinear
            R_mag = inf; 
            turnSign = 0; 
        end
        RProfileLap(i) = R_mag; 
        TSignLap(i) = turnSign; 
        ARProfileLap(i) = area2;
    end
    
    %% Drive Cycle Generator Path
    nLaps = 1;
    xresMCP_laps = repmat(xresMCP, nLaps, 1);
    yresMCP_laps = repmat(yresMCP, nLaps, 1);
    RProfile     = repmat(RProfileLap,  nLaps, 1);
    TSignProfile = repmat(TSignLap,     nLaps, 1);
    
    % Compute current lap length & Scale
    dx_seg = diff(xresMCP_laps); dy_seg = diff(yresMCP_laps);
    segmentLengths = sqrt(dx_seg.^2 + dy_seg.^2);
    lapLength = sum(segmentLengths);
    targetLap = 5078 * nLaps; % - 5078 for aragon
    scale_factor = targetLap / lapLength;
    
    % Apply scaling
    xresMCP_laps = xresMCP_laps * scale_factor;
    yresMCP_laps = yresMCP_laps * scale_factor;
    
    % Recompute lengths
    dx_seg = diff(xresMCP_laps); dy_seg = diff(yresMCP_laps);
    segmentLengths = sqrt(dx_seg.^2 + dy_seg.^2);
    
    %% Output Packaging
    trackDataOut.xresMCP_laps = xresMCP_laps;
    trackDataOut.yresMCP_laps = yresMCP_laps;
    trackDataOut.RProfile = RProfile;
    trackDataOut.TSignProfile = TSignProfile;
    trackDataOut.segmentLengths = segmentLengths;
    trackDataOut.zt = zt;
    trackDataOut.xt = xt; 
    trackDataOut.yt = yt;
    trackDataOut.xin = xin; trackDataOut.xout = xout;
    trackDataOut.yin = yin; trackDataOut.yout = yout;
    trackDataOut.xresMCP = xresMCP; 
    trackDataOut.yresMCP = yresMCP;
    trackDataOut.finalStepLocs = finalStepLocs;
    trackDataOut.scale_factor = scale_factor;
    trackDataOut.name = name;
end