function [xF, lenLeg, dthLeg_dt] = calculateKinematics(X, params) 
    % unpack variables
    [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X); %#ok<ASGLU>
    [m,I,k,len0,g,b] = unpackParams(params); %#ok<ASGLU>
    stanceBool = isInStance(X, params);
    
    % xF = foot x location
    xFStance = x + y.*tan(thLeg); 
    lenLegStance = sqrt((x - xFStance).^2 + y.^2);
    dthLeg_dtStance = (-y.*dxdt -(xFStance - x).*dydt)./lenLegStance.^2;

    xFflight = x + len0.*sin(thLeg); 
    lenLegFlight = len0*ones(size(x));
    dthLeg_dtFlight = zeros(size(x));
    
    xF = xFStance.*stanceBool + xFflight.*(1-stanceBool);
    lenLeg = lenLegStance.*stanceBool + lenLegFlight.*(1-stanceBool);
    dthLeg_dt = dthLeg_dtStance.*stanceBool + dthLeg_dtFlight.*(1-stanceBool);
end