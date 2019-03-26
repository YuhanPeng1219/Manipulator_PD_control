function Problem3(initial_states)
% The function is used to run simulation and generate plots. The input is
% the initial state of manipulator.
%
%% ***************** Initial funcition handles and states *****************
% trajectory generator
trajhandle = @tra_generate;

% controller
controlhandle = @controller;

%% ****************** Define the parameter of lynx ************************
param = manipulatorParameter();

%% ****************** Define the real configuration ***********************
q = initial_states(1, :);
qd = initial_states(2, :);
qdd = initial_states(3, :);
q_restore = zeros(2, 200);
qd_restore = zeros(2, 200);
qdes_restore = zeros(2, 200);
qddes_restore = zeros(2, 200);
V_restore = zeros(1, 200);
Vdot_restore = zeros(1, 200);
global e edot
t_matrix = 0:0.05:9.95;

%% ****************** Prepare for simulation ******************************
fprintf('Initializing....')
max_iter  = 5000;      % max iteration
max_time = 10;
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
t      = starttime; % current time
x = zeros(1,4);        % initial state vector
x(1:2) = q; x(3:4) = qd;
%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....')
% Main loop
for i = 1:max_iter
    
    timeint = t:tstep:t+cstep;
    
    tic;
    % Iterate over each quad
    if i == 1
        % Initialize quad plot
        [q_des, qd_des, ~]  = trajhandle(0);
        Robot_plot(q_des, false);
        h_title = title(sprintf('iteration: %d, time: %4.2f', i, t));
        hold on;
        drawnow;
        % Run simulation
    else
        [q_des, qd_des, qdd_des]  = trajhandle(t);
        Robot_plot(q_des, false);
        h_title = title(sprintf('iteration: %d, time: %4.2f', i, t));
        hold on;
        qs.pos = q; qs.vel = qd;
        qs.pos_des = q_des; qs.vel_des = qd_des; qs.acc_des = qdd_des;
        [tsave,xsave] = ode45(@(t,s)controlhandle(t,s,qs,param), timeint, x);
        q = xsave(6,1:2);
        qd = xsave(6,3:4);
        x = xsave(6,:);
            % compute V and Vdot
    [V, Vdot] = Vcompute(e, edot, 10, 5);
    
    % save the data
    q_restore(:, i) = q;
    qd_restore(:, i) = qd;
    qdes_restore(:, i) = q_des;
    qddes_restore(:, i) = qd_des;
    V_restore(:,i) = V;
    Vdot_restore(:,i) = Vdot;
    end
    % update real figure
    Robot_plot(q, true);
    hold off;
    drawnow;
   
    t = t + cstep; % Update simulation time
   
    % Check termination criteria
    if t > max_time
        break
    end
    

end
%% ************************* Post processing *************************
% Figure compare joint angle
figure;

% joint 1
subplot(1,2,1);
plot(t_matrix,q_restore(1,:));
set(gcf,'Color','w'); 
grid on; hold on;
plot(t_matrix,qdes_restore(1, :),'--')
xlabel('time [sec]'); ylabel('q(t)');
legend('q1','q1des');
title('Joint angle control performance'); 

% joint 2
subplot(1,2,2);
plot(t_matrix,q_restore(2,:));
set(gcf,'Color','w'); 
grid on; hold on;
plot(t_matrix,qdes_restore(2, :),'--')
xlabel('time [sec]'); ylabel('q(t)');
legend('q2','q2des');
title('Joint angle control performance'); 

% Figure compare joint angular velocity
figure;

% Joint 1
subplot(1,2,1);
plot(t_matrix,qd_restore(1,:));
set(gcf,'Color','w'); 
grid on; hold on;
plot(t_matrix,qddes_restore(1, :),'--')
xlabel('time [sec]'); ylabel('qd(t)');
legend('qd1','qd1des');
title('Joint velocity control performance'); 

% Joint 2
subplot(1,2,2);
plot(t_matrix,qd_restore(2,:));
set(gcf,'Color','w'); 
grid on; hold on;
plot(t_matrix,qddes_restore(2, :),'--')
xlabel('time [sec]'); ylabel('qd(t)');
legend('qd2','qd2des');
title('Joint velocity control performance'); 

% Figure for V and Vdot versus time
figure;

% V
subplot(1,2,1);
plot(t_matrix,V_restore(1,:));
set(gcf,'Color','w'); 
grid on; 
xlabel('time [sec]'); ylabel('V');
title('V versus time');

% Vdot
subplot(1,2,2);
plot(t_matrix,Vdot_restore(1,:));
set(gcf,'Color','w'); 
grid on; 
xlabel('time [sec]'); ylabel('Vdot');
title('Vdot versus time'); 
end


%% ***************** Function to initialize parameters ********************
function param = manipulatorParameter()
   param.m1 = 1;
   param.m2 = 1;
   param.a1 = 1;
   param.a2 = 2;
   param.grav = 9.8;
end

%% ***************** Function to generate trajectory ********************
function [pos, vel, acc] = tra_generate(t)
% TRAJECTORY GENERATOR
%   t      - the current time
%   pos    - the desired joint position
%   vel    - the desired joint velocity
%   acc    - the desired joint acceleration
pos = [sin(t), -cos(t)];
vel = [cos(t), sin(t)];
acc = [-sin(t), cos(t)];
end

%% ***************** Function to control dynamics ********************
function q_pva = controller(t, x, qs, param)
% CONTROLLER manipulator controller
%   qs     - the current states: qs.pos, qs.vel;
%            the desired states: qs.pos_des, qs.vel_des, qs.acc_des;
%   t      - time, unit is second;
%   param - output from manipulator() storing all robot parameters;
%   tau    - 4 x 1, controller output.
%   Compute the desired controls

global e edot

% initialize output
q_pva = zeros(4,1);

% Turning gains 
Kd = sqrt(2)*eye(2);
Kp = 1*eye(2);

m1 = param.m1;
m2 = param.m2;

a1 = param.a1;
a2 = param.a2;

g = param.grav;

q1 = x(1);
q2 = x(2);

q1dot = x(3);
q2dot = x(4);

e = x(1:2) - qs.pos_des';
edot = x(3:4) - qs.vel_des';

% calculate the M,C,and N based on the calculation of dynamics
M = [m1*a1^2+m2*(a1^2+a2^2+2*a1*a2*cos(q2)), m2*(a2^2+a1*a2*cos(q2));
     m2*(a2^2+a1*a2*cos(q2)), m2*a2^2];
C = [-m2*a1*a2*sin(q2)*q2dot, m2*a1*a2*sin(q2)*q1dot-m2*a1*a2*sin(q2)*q2dot;
     m2*a1*a2*sin(q2)*q1dot, 0];
N = [m1*g*a1*cos(q1)+m2*g*(a1*cos(q1)+a2*cos(q1+q2));
     m2*g*a2*cos(q1+q2)];
                                                                                                                             
feed_value = qs.acc_des' - Kd*edot - Kp*e;

% compute input value
tau = (M * (feed_value) + C*x(3:4) + N);

% Uncomment this if want to add torque limits
% if tau > 10
%     tau = 10;
% elseif tau < -10
%     tau = -10;
% end

q_pva(1:2) = x(3:4);
q_pva(3:4) = (M\(tau - (C*x(3:4) + N)));
end

%% ***************** Function compute V *********************
function [V, Vdot] = Vcompute(e, edot, Kp, Kd)
    V = 0.5*Kp*e(1)^2+0.5*Kp*e(2)^2+0.5*edot(1)^2+0.5*edot(2)^2;
    Vdot = -Kd * (edot(1)^2+edot(2)^2);
end
%% ***************** Function to plot robot motion ********************
function Robot_plot(q, real)
   if real == 1
      scatter(cos(q(1)), sin(q(1)), 140, 'filled','r');
      hold on;
      scatter(cos(q(1))+2*cos(q(1)+q(2)), sin(q(1)) + 2*sin(q(1)+q(2)), ...
                                                        140, 'filled','r');
      line([0;cos(q(1))], [0;sin(q(1))], 'Linewidth', 3, 'color', [1,0,0]);
      line([cos(q(1)) + 2*cos(q(1)+q(2));cos(q(1))], ...
           [sin(q(1)) + 2*sin(q(1)+q(2));sin(q(1))], 'Linewidth', 3, 'color', [1,0,0]);
      grid on;
      axis([-5 5 -5 5]);
      xlabel('x [m]'); ylabel('y [m]');
   else
      scatter(cos(q(1)), sin(q(1)), 140, 'filled', 'b');
      hold on;
      scatter(cos(q(1)) + 2*cos(q(1)+q(2)), sin(q(1)) + 2*sin(q(1)+q(2)), ...
                                                       140, 'filled', 'b');
      line([0;cos(q(1))], [0;sin(q(1))], 'Linewidth', 3, 'color', [0,0,1]);
      line([cos(q(1)) + 2*cos(q(1)+q(2));cos(q(1))], ...
           [sin(q(1)) + 2*sin(q(1)+q(2));sin(q(1))], 'Linewidth', 3, 'color', [0,0,1]);
   end
end
