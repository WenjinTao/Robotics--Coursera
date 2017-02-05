% ==============================================================
% utility function for packing/unpacking/passing data
% ==============================================================
function [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X)
    
    % There's a weird situation where X is transposed in ode45...
    if(size(X,2)~=7)
        X = X';
    end
 
    x    = X(:,1);    y = X(:,2); phi     = X(:,3); thLeg     = X(:,4);
    dxdt = X(:,5); dydt = X(:,6); dphi_dt = X(:,7);
end