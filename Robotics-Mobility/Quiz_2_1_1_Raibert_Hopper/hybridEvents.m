function [zeroCrossing,isterminal,direction] = hybridEvents(~,X, params, ~)

    % grab state
    [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X); %#ok<ASGLU>
    [m,I,k,len0,g,b] = unpackParams(params); %#ok<ASGLU>
     
    % form errors via kinematics
    groundImpact = y;
    apexHeight   = dydt;
    touchdown    = len0 - y./cos(thLeg);
    liftoff      = len0 - y./cos(thLeg);
    
    % Locate the time when height passes through zero in a 
    % decreasing direction and stop integration.
    zeroCrossing = [groundImpact, apexHeight, touchdown, liftoff];  
    isterminal   = [1           , 1         , 1        , 1      ];             
    direction    = [-1          ,-1         , 1        ,-1      ];
end