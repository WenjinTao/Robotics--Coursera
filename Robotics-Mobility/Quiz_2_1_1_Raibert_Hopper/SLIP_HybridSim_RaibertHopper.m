function [] = SLIP_HybridSim_RaibertHopper(duration,gain,hObject,probnum)

    % --- parameters for eom ---
    params.m    = 1;
    params.I    = 0.25;
    params.k    = 5000;
    params.len0 = 0.75;
    params.g    = 9.81;
    params.b    = 9; % was 10
    
    % --- initial conditions ---
    x0         = -0.5; 
    y0         = 0.8;
    phi0       = 0; 
    dxdt0      = 0;
    thLeg0     = 0;
    dydt0      = -0.01;
    dphi_dt0   = 0;
    ic = [x0,y0,phi0,thLeg0,dxdt0,dydt0,dphi_dt0];
    
     % --- control variables ---
    % forward speed
    
    if probnum ==1 || probnum==3
        control.dxdtDesired = 0;
    elseif probnum ==2
        control.dxdtDesired = 2;
    else
        control.dxdtDesired = 1.5;
    end
    control.k_dxdt = gain;%0.1; 
    % pitch
    control.phiDesired = pi/12;
    control.kp_phi = 3; % was 3
    control.kd_phi = 3; % was 3
    % hopping height
    control.thrustDuration = duration;%0.05; % was 0.1
    control.thrust = 20; % was 20
    
    % --- simulation parameters ---
    if probnum==1
        tmax = 6;
    elseif probnum ==2
        tmax = 4;
    elseif probnum==3
        tmax = 8;
    else
        tmax = 4;
    end
    % --- video ---
    createVideo = false;
    videoName = '~/Desktop/test.avi';
    
    % ==============================
    % bookkeeping
    bookkeeping.touchdownTime = 0;
    bookkeeping.liftoffTime = 0;

    % run simulation
    errorEvents = [1]; %#ok<NBRAK>
    [t,X] = runSim(@dynamics, @hybridEvents, @handleEvent, ...
        tmax, params, bookkeeping, ic, errorEvents,control,probnum);
    % ==============================
    
    % visualize data
%     plotStates(t,X, params, bookkeeping);
%     plotEnergetics(t,X, params, bookkeeping);
    if probnum ==1
        plotStates1(t,X, params, bookkeeping,hObject);
        drawPicture1(t,X, params, bookkeeping, createVideo, videoName,hObject); 
    elseif probnum==2
        plotStates2(t,X, params, bookkeeping,hObject);
        drawPicture2(t,X, params, bookkeeping, createVideo, videoName,hObject);
    elseif probnum==3;
        plotStates3(t,X, params, bookkeeping,hObject);
        drawPicture3(t,X, params, bookkeeping, createVideo, videoName,hObject);
    else
        plotStates4(t,X, params, bookkeeping,hObject);
        drawPicture4(t,X, params, bookkeeping, createVideo, videoName,hObject);
    end
end
 
% ==============================================================
% Encode hybrid dynamics
% ==============================================================
function dXdt = dynamics(t,X,params, bookkeeping, control,probnum)
 
    % unpack parameters, state, and control vars
    [m,I,k,len0,g,b] = unpackParams(params);
    [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X);  %#ok<ASGLU>
    [touchdownTime, liftoffTime] = unpackBookkeeping(bookkeeping); %#ok<ASGLU>
    [dxdtDesired, k_dxdt, phiDesired, kp_phi, kd_phi, ...
        thrustDuration, thrust] = unpackControl(control); %#ok<ASGLU>
    
    % set up state (but don't include thLeg until later)
    q    = [x;       y;     phi]; %#ok<NASGU>
    dqdt = [dxdt; dydt; dphi_dt];
    [xF, lenLeg, dthLeg_dt] = calculateKinematics(X, params);
     
    % account for hybrid state
    isInStanceBool = isInStance(X, params);
    if(isInStanceBool)
        
        if((t - touchdownTime) < thrustDuration)
            Fr   = thrust*(dydt/sqrt(dydt^2));
        else
            Fr   = 0;
        end
        if probnum ==3 || probnum ==4
            Tphi = -1*pitchController(phi,phiDesired,dphi_dt);
        else
            Tphi = -1*(-kp_phi*(phi - phiDesired) - kd_phi*dphi_dt);
        end
        Fx_friction = b*dxdt;
        Fy_friction = b*dydt;
    else
        % in flight, motors can't act on body 
        % (except to change massless leg angle)
        Fr   = 0;
        Tphi = 0;
        Fx_friction = 0;
        Fy_friction = 0;
    end

    % set up manipulation equation matrices
    M = [m, 0, 0;
         0, m, 0;
         0, 0, I];
    C =  0;
    N = [(x-xF)*k*(1 - (len0/lenLeg));
         y*k*(1 - (len0/lenLeg)) + m*g;
         0];
    Tau = [Fr*(x-xF)/lenLeg - Tphi*y/(lenLeg^2) - Fx_friction;
           Fr*y/lenLeg + Tphi*(x-xF)/(lenLeg^2) - Fy_friction;
          -Tphi];
       
    % deal with the case in which stance normal force goes
    %   to zero due to actuation
    if(isInStanceBool)
        ddqdt = M\(Tau - C*dqdt - N);
        verticalAcceleration = ddqdt(2);
        if(verticalAcceleration < -g)
            N   = [0; m*g; 0];
            Tau = [0;   0; 0];
            dthLeg_dt = 0;
        end
    end
     
    % form 1st order system - including leg angle
    dXdt = [
            dqdt;
            dthLeg_dt;
            M\(Tau - C*dqdt - N);
           ];
end

% --------------------------------------------------------------
function [X,params,bookkeeping] = ...
    handleEvent(event,t,X,params,bookkeeping, control) 

    [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X); 
    [m,I,k,len0,g,b] = unpackParams(params); %#ok<ASGLU>
    [dxdtDesired, k_dxdt, phiDesired, kp_phi, kd_phi, ...
        thrustDuration, thrust] = unpackControl(control); %#ok<ASGLU>
    isInStanceBool = isInStance(X, params);

    if(event == 3) % touchdown 
       % record touchdown time
       bookkeeping.touchdownTime = t(end);
    elseif(event == 4) % liftoff
        % record liftoff time
        bookkeeping.liftoffTime = t(end);
    elseif(event == 2) % apex height reached
        
        Tstance = bookkeeping.liftoffTime ...
                - bookkeeping.touchdownTime; 
        
        xF = (dxdt(end)*Tstance)/2 + k_dxdt*(dxdt(end) - dxdtDesired);      
       
        if(~isInStanceBool(end))
            if(abs(xF/len0)<= 1)
                thLegDesired = asin(xF/len0);
                if(y(end)>= len0*cos(thLegDesired))
                    thLeg(end) = thLegDesired;
                else
                    thLeg(end) = acos(y(end)/len0);
                end
            else
                thLeg(end) = (pi/2)*sign(xF);
            end
        end
    end
    
    X = [x,y,phi,thLeg,dxdt,dydt,dphi_dt];
end

% ==============================================================
% utility functions for packing/unpacking/passing data
% ==============================================================
function ...
    [dxdtDesired, k_dxdt, phiDesired, kp_phi, kd_phi, ...
     thrustDuration, thrust] = unpackControl(control)
    
    % forward speed
    dxdtDesired = control.dxdtDesired;
    k_dxdt = control.k_dxdt; 
    % pitch
    phiDesired = control.phiDesired;
    kp_phi = control.kp_phi;
    kd_phi = control.kd_phi;
    % hopping height
    thrustDuration = control.thrustDuration;
    thrust = control.thrust;
end

