function [t_out, s_out] = simulation_2d(controlhandle, trajhandle)

video = false;

params = sys_params;

% real-time
real_time = true;

%% **************************** FIGURES *****************************
disp('Initializing figures...')
if video
  video_writer = VideoWriter('test_control_2d.avi', 'Uncompressed AVI');
  open(video_writer);
end
h_fig = figure;
sz = [790 607]; % figure size
screensize = get(0,'ScreenSize');
xpos = ceil((screensize(3)-sz(1))/2); % center the figure on the screen horizontally
ypos = ceil((screensize(4)-sz(2))/2); % center the figure on the screen vertically
set(h_fig, 'Position', [xpos ypos sz])

h_3d = subplot(3,3,[1,2,4,5,7,8]);
axis equal
grid on
view(90,0);
ylabel('y [m]'); zlabel('z [m]');

% h_2d = subplot(1,2,2);
% plot_2d = plot(h_2d, 0, 0);
% grid on;
% xlabel('t [s]'); ylabel('z [m]');

quadcolors = lines(1);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
t_total  = 5;             % Total simulated time
tstep    = 0.01;          % this determines the time step at which the solution is given
cstep    = 0.05;          % image capture time interval
max_iter = t_total/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
err = []; % runtime errors
% Get start and stop position
des_start = trajhandle(0,[]);
des_stop  = trajhandle(inf,[]);

% Get boundary
d_state = nan(max_iter,2);
for iter = 1:max_iter
    dd = trajhandle(cstep*iter,[]);
    d_state(iter,:) = dd.pos(1:2)';
end
y_lim = [min(d_state(:,1)) - 0.1, max(d_state(:,1)) + 0.1];
z_lim = [min(d_state(:,2)) - 0.1, max(d_state(:,2)) + 0.1];
if(4*(z_lim(2) - z_lim(1)) < y_lim(2) - y_lim(1))
    z_lim(1) = z_lim(1) - (y_lim(2) - y_lim(1))/8;
    z_lim(2) = z_lim(2) + (y_lim(2) - y_lim(1))/8;
end
stop_pos = des_stop.pos;
x0        = [des_start.pos; 0; des_start.vel; 0];
xtraj     = nan(max_iter*nstep, length(x0));
ttraj     = nan(max_iter*nstep, 1);

x         = x0;        % state

pos_tol = 0.01;
vel_tol = 0.03;
ang_tol = 0.05;

%% ************************* RUN SIMULATION *************************
disp('Simulation Running....')
% Main loop
for iter = 1:max_iter

  timeint = time:tstep:time+cstep;

  tic;
  % Initialize quad plot
  if iter == 1
    subplot(3,3,[1,2,4,5,7,8]);
    quad_state = simStateToQuadState(x0);
    QP = QuadPlot(1, quad_state, params.arm_length, 0.05, quadcolors(1,:), max_iter, h_3d);
    ylim(y_lim); zlim(z_lim);
    quad_state = simStateToQuadState(x);
    QP.UpdateQuadPlot(quad_state, time);
    h_title = title(h_3d, sprintf('iteration: %d, time: %4.2f', iter, time));
  end

  % Run simulation
  [tsave, xsave] = ode45(@(t,s) sys_eom(t, s, controlhandle, trajhandle, params), timeint, x);
  x = xsave(end, :)';

  % Save to traj
  xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
  ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

  % Update quad plot
  quad_state = simStateToQuadState(x);
  QP.UpdateQuadPlot(quad_state, time + cstep);
  subplot(3,3,[1,2,4,5,7,8]);
  ylim(y_lim); zlim(z_lim);
  set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
  time = time + cstep; % Update simulation time
  if video
    writeVideo(video_writer, getframe(h_fig));
  end
    subplot(3,3,3)
    plot(ttraj(1:iter*nstep), xtraj(1:iter*nstep,1));
    xlabel('t [s]'); ylabel('y [m]');
    grid on;
    subplot(3,3,6)
    plot(ttraj(1:iter*nstep), xtraj(1:iter*nstep,2));
    xlabel('t [s]'); ylabel('z [m]');
    grid on;
    subplot(3,3,9)
    plot(ttraj(1:iter*nstep), 180/pi*xtraj(1:iter*nstep,3));
    grid on;
    xlabel('t [s]'); ylabel('\phi [�]');
%
%     figure(2);
%     subplot(3,1,3)
%     plot(ttraj(1:iter*nstep), 180/pi*xtraj(1:iter*nstep,3));
%     grid on;
%     subplot(6,1,4)
%     plot(ttraj(1:iter*nstep), xtraj(1:iter*nstep,4));
%     grid on;
%     subplot(6,1,5)
%     plot(ttraj(1:iter*nstep), xtraj(1:iter*nstep,5));
%     grid on;
%     subplot(6,1,6)
%     plot(ttraj(1:iter*nstep), 180/pi*xtraj(1:iter*nstep,6));
%     grid on;

  t = toc;
  % Check to make sure ode45 is not timing out
  if(t > cstep*50)
    err = 'Ode45 Unstable';
    break;
  end

  % Pause to make real-time
  if real_time && (t < cstep)
    pause(cstep - t);
  end

  % Check termination criteria
  if time > t_total - 0.001 
    if norm(x(1:2) - stop_pos) < pos_tol && norm(x(4:5)) < vel_tol && abs(x(3)) < ang_tol
      err = [];
      break
    elseif norm(x(1:2) - stop_pos) > pos_tol
      err = 'Did not reach goal';
    elseif norm(x(4:5)) > vel_tol
      err = 'Velocity not close to zero';
    elseif abs(x(3)) > ang_tol
      err = 'Final angle not close to zero';
    end
  end
end
disp('Simulation done');
if video
  close(video_writer);
end

if ~isempty(err)
  disp(['Error: ', err]);
  t_out = [];
  s_out = [];
else
  disp(['Final time: ', num2str(time), ' sec']);
  t_out = ttraj(1:iter*nstep);
  s_out = xtraj(1:iter*nstep,:);
  %disp(['Stopping distance [m]: ', num2str(max(s_out(:,1)))]);
  %disp(['Dropping distance [m]: ', num2str(abs(min(min(s_out(:,2)),0)))]);
end
end
