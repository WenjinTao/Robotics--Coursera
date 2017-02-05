% ==============================================================
% Simulate hybrid dynamical system
% ==============================================================
function [t,y] = runSim(dynamics,events, handleEvent, ...
    tmax, params, bookkeeping, ic, errorEvents, control,probnum)

    t = [];
    y = [];
    tLatest = 0;
    maxTimeStep = 1e-2;
 
    while(tLatest < tmax)

        tspan = [tLatest, tmax];
        
        % set up event detection
        options = odeset('Events', ...
                    @(t,y) events(t,y, params, bookkeeping), ...
                    'MaxStep', maxTimeStep);
         
        % Run simulation
        % t  is time
        % y  is soln 
        % tE is time events occured
        % yE is soln at event time
        % iE is index of vanishing event fxn
        [tChart,yChart,tE,yE,iE]= ode45(...
                        @(t,y) dynamics(t,y,params, bookkeeping, control,probnum), ...
                        tspan, ic, options); %#ok<ASGLU>
        
         % Record data
        t = [t; tChart]; %#ok<AGROW>
        y = [y; yChart]; %#ok<AGROW>  
        tLatest = t(end);
        
         
        % Find the first even that occured (if any), record the data up to
        %   it and make sure it's not an error (falling over).  If so then 
        %   stop simulating.
        if(~isempty(iE))
            % check for fatal events
            for i = 1:length(iE)
                if(~isempty(find(errorEvents == iE(i),1)))
                    break;
                end
            end
            % lots of time the ic is an event, in which case look at 2nd
            if(length(iE)>1)
                eventIdx = 2;
            else
                eventIdx = 1;
            end
            % deal with event
            event = iE(eventIdx);
            [y,params,bookkeeping] = ...
               handleEvent(event,t,y,params,bookkeeping, control);
        end
        
        % record initial conditions for next time
        ic = y(end,:);
    end
end