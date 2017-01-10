function [t_out, z_out] = height_control(trajhandle, controlhandle)

addpath('utils');

video = false;
video_filename = 'height_control.avi';

params = sys_params;

% real-time
real_time = true;

%% **************************** FIGURES *****************************
disp('Initializing figures...')
if video
  video_writer = VideoWriter(video_filename, 'Uncompressed AVI');
  open(video_writer);
end
h_fig = figure;
sz = [1000 600]; % figure size
screensize = get(0,'ScreenSize');
xpos = ceil((screensize(3)-sz(1))/2); % center the figure on the screen horizontally
ypos = ceil((screensize(4)-sz(2))/2); % center the figure on the screen vertically
set(h_fig, 'Position', [xpos ypos sz])

h_3d = subplot(1,2,1);
axis equal
grid on
view(0,0);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');

h_2d = subplot(1,2,2);
plot_2d = plot(h_2d, 0, 0);
grid on;
xlabel('t [s]'); ylabel('z [m]');

quadcolors = lines(1);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
max_iter  = 100;       % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
% Get start and stop position
des_start = trajhandle(0);
des_stop  = trajhandle(inf);
stop_pos  = des_stop(1);
x0        = des_start;
xtraj     = nan(max_iter*nstep, length(x0));
ttraj     = nan(max_iter*nstep, 1);

x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
disp('Simulation Running....')
% Main loop
for iter = 1:max_iter

  timeint = time:tstep:time+cstep;

  tic;
  % Initialize quad plot
  if iter == 1
    subplot(1,2,1);
    quad_state = simStateToQuadState(x0);
    QP = QuadPlot(1, quad_state, params.arm_length, 0.05, quadcolors(1,:), max_iter, h_3d);
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
  subplot(1,2,1)
  quad_state = simStateToQuadState(x);
  QP.UpdateQuadPlot(quad_state, time + cstep);
  set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
  time = time + cstep; % Update simulation time

  set(plot_2d, 'XData', ttraj(1:iter*nstep), 'YData', xtraj(1:iter*nstep,1));
  if video
    writeVideo(video_writer, getframe(h_fig));
  end

  t = toc;
  % Check to make sure ode45 is not timing out
  if(t > cstep*50)
    err = 'Ode solver took too long for a step. Maybe the controller is unstable.';
    disp(err);
    break;
  end

  % Pause to make real-time
  if real_time && (t < cstep)
    pause(cstep - t);
  end

end
% Check termination criteria
if norm(stop_pos - x(1)) < pos_tol && norm(x(2)) < vel_tol
  err = [];
else
  err = 'Did not converge';
end

disp('Simulation done');
if video
  close(video_writer);
end

if ~isempty(err)
  disp(['Error: ', err]);
end
t_out = ttraj(1:iter*nstep);
z_out = xtraj(1:iter*nstep,1);

end
