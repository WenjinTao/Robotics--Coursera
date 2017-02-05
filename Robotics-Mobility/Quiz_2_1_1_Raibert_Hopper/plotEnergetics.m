function [] = plotEnergetics(t,X, params, bookkeeping)

    % unpack variables
    [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X); %#ok<ASGLU>
    [m,I,k,len0,g,b] = unpackParams(params); %#ok<ASGLU>
    [touchdownTime, liftoffTime] = unpackBookkeeping(bookkeeping); %#ok<ASGLU>
    [xF, lenLeg, dthLeg_dt] = calculateKinematics(X, params) ; %#ok<ASGLU>
      
    % calculate energy
    K = (m/2)*(dxdt.^2 + dydt.^2) + (I/2)*(dphi_dt.^2);
    Vgrav = m*g*y;
    Vspring = isInStance(X,params)*(k/2).*(lenLeg - len0).^2;
    V = Vgrav + Vspring;

    % visualizations
    figure; hold on;
    plot(t,K);
    plot(t,V);
    plot(t,Vgrav);
    plot(t,Vspring);
    plot(t,K+V);
    legend('K','V','Vgrav','Vspring','K+V');
%     legend('K','V','K+V');
end