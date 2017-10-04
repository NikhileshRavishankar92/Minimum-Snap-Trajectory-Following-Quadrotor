function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

%% Prams for line trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Angles
%control_param = [(29-4.25/8 - 2.95/8 - 2.15*1.8/8 - 1.55 * 4.8/8),13*2.2,(29-4.25/8 - 2.95/8 - 1.55* 4.8/8),13*2.2,9,4];

% 2D case [3,1.64843752100113,11.3123279427234,36.4716338194995,3.26830632348692,18.7282693530237]

% Accelerations
%k_v_new = [17.95 17.95 16];  
%k_p_new = [4.1/1.9 4.1/1.9 2]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

%% Params for helix trajectory
% Angles
%control_param = [(29-4.25/8 - 2.95/8 - 2.15*1.8/8 - 1.55 * 4.8/8 - 1.5*7/8),13*2.2*2.2,(29-4.25/8 - 2.95/8 - 1.55* 4.8/8 - 1.5*7/8),13*2.2*2.2,12,4];
% Accelerations
%k_v_new = [(17.95*0.8*0.65) (17.95*0.8*0.65) 16];  
%k_p_new = [4.1/1.9 4.1/1.9 12];  

%%

% =================== Your code goes here ===================

% Angles
control_param = [20, 79, 25, 69,14,8] ; % [(29-4.25/8 - 2.95/8 - 2.15*1.8/8 - 1.55 * 4.8/8 - 1.5*7/8 + 4),13*2.2*2.2*1.6 - 2 ,(29-4.25/8 - 2.95/8 - 1.55* 4.8/8 - 1.5*10/8),13*2.2*2.2 + 7,18,8]

% Accelerations
k_v_new = [14 (9.5 + 1/8) 25]; %[(17.95*0.8*0.96 ) (17.95*0.8*1.15 + 1.0) 29]
k_p_new = [16.95 18.5 100];  % [4.9*4.1/1.9 , 4.8*4.1/0.9 + 2.5, 160]  


% t_denominator = sqrt(900 + 3600*t^2 + 900*t^4);
% t_cap = [30/t_denominator; -60*t/t_denominator; 30*t^2/t_denominator];
% t_cap = des_state.vel/norm(des_state.vel);
% %t_cap = des_state.vel/4;
% 
% n_denominator = sqrt(1 + 21*t^2 + 86*t^4 + 89*t^6 + 21*t^8 + 4*t^10);
% n_cap = [(-2*t - t^3)/n_denominator; (-1 -8*t^2 -3*t^4)/n_denominator;(t + 6*t^3 + 2*t^5)/n_denominator];
% n_cap = des_state.acc/norm(des_state.acc);
% %n_cap = des_state.acc/4;
% 
% b_cap = [0 -t_cap(3) t_cap(2);t_cap(3) 0 -t_cap(1);-t_cap(2) t_cap(1) 0] * n_cap;
% 
% pos_new = ((des_state.pos - state.pos)' * n_cap) * n_cap + ((des_state.pos - state.pos)' * b_cap) * b_cap;
% vel_new = des_state.vel - state.vel;


r_traj_new(1) = des_state.acc(1) + k_v_new(1) * (des_state.vel(1) - state.vel(1)) + k_p_new(1) * (des_state.pos(1) - state.pos(1));
r_traj_new(2) = des_state.acc(2) + k_v_new(2) * (des_state.vel(2) - state.vel(2)) + k_p_new(2) * (des_state.pos(2) - state.pos(2));
r_traj_new(3) = des_state.acc(3) + k_v_new(3) * (des_state.vel(3) - state.vel(3)) + k_p_new(3) * (des_state.pos(3) - state.pos(3));

    

% Thrust
%F = params.mass * (params.gravity + control_param(1) * (0 - state.vel(3)) + control_param(2) * (des_state.pos(3) - state.pos(3)));
F = params.mass * (params.gravity + r_traj_new(3));


theta_c = (1/params.gravity) * (r_traj_new(1) * cos(des_state.yaw) + r_traj_new(2) * sin(des_state.yaw));
phi_c = (1/params.gravity) * (r_traj_new(1) * sin(des_state.yaw) - r_traj_new(2) * cos(des_state.yaw));

% Moment
M = zeros(3,1);
u2 = zeros(3,1);

u2(1) = (control_param(1) * (-state.omega(1)) + control_param(2) * (phi_c - state.rot(1))); 
u2(2) = (control_param(3) * (-state.omega(2)) + control_param(4) * (theta_c - state.rot(2)));
u2(3) = (control_param(5) * (des_state.yawdot - state.omega(3)) + control_param(6) * (des_state.yaw - state.rot(3)));

M = params.I * u2;


% =================== Your code ends here ===================

end
