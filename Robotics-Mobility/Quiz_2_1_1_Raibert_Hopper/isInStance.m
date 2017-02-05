function stanceBool = isInStance(X, params)

    % There's a weird situation where X is transposed in ode45...
    if(size(X,2)~=7)
        X = X';
    end

    % grab state
    [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X); %#ok<ASGLU>
    [m,I,k,len0,g,b] = unpackParams(params); %#ok<ASGLU>
    
    % determine if in stance
    stanceBool = zeros(size(X,1),1);
    stanceBool(find(len0 > y./cos(thLeg))) = 1; %#ok<FNDSB>
    stanceBool(find(cos(thLeg) == 0)) = 0; %#ok<FNDSB>
end