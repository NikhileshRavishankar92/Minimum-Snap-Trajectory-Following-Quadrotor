function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 C_xx C_xy C_xz

if nargin > 2
    disp(['Number of arguments = ', num2str(nargin)]);
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2); % 2 comes from the fact that the speed of the quadrotor is 0.5 m/s, d0 is the total time
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints';
    
%     desired_state.pos = zeros(3,1);
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
    
   % disp(traj_time);
   % disp(d0);
    
    %% X position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
        k1 = traj_time(2);
        k2 = traj_time(3) ;
        k3 = traj_time(4);
        k4 = traj_time(5) ;
        
        A_x = zeros(32,32);
        C_x = zeros(32,1);
        y_x = zeros(32,1);
        y_x(1) = waypoints0(1,1);
        y_x(5) = waypoints0(2,1);
        y_x(6) = waypoints0(2,1);
        y_x(13) = waypoints0(3,1);
        y_x(14) = waypoints0(3,1);
        y_x(21) = waypoints0(4,1);
        y_x(22) = waypoints0(4,1);
        y_x(29) = waypoints0(5,1);

        A_x(1,1) = 1;
        A_x(2,2) = 1;
        A_x(3,3) = 2;
        A_x(4,4) = 6;
        A_x(5,1:8) = [1 k1  k1^2 k1^3 k1^4 k1^5 k1^6 k1^7];
        A_x(6,9:16)= [1 k1  k1^2 k1^3 k1^4 k1^5 k1^6 k1^7];
        A_x(7,1:16) = [0 1 2*k1 3*k1^2 4*k1^3 5*k1^4 6*k1^5 7*k1^6 0 -1 -2*k1 -3*k1^2 -4*k1^3 -5*k1^4 -6*k1^5 -7*k1^6];
        A_x(8,1:16) = [0 0 2 6*k1 12*k1^2 20*k1^3 30*k1^4 42*k1^5 0 0 -2 -6*k1 -12*k1^2 -20*k1^3 -30*k1^4 -42*k1^5];
        A_x(9,1:16) = [0 0 0 6 24*k1 60*k1^2 120*k1^3 210*k1^4 0 0 0 -6 -24*k1 -60*k1^2 -120*k1^3 -210*k1^4];
        A_x(10,1:16) = [0 0 0 0 24 120*k1 360*k1^2 820*k1^3 0 0 0 0 -24 -120*k1 -360*k1^2 -820*k1^3];
        A_x(11,1:16) = [0 0 0 0 0 120 720*k1 2460*k1^2 0 0 0 0 0 -120 -720*k1 -2460*k1^2];
        A_x(12,1:16) = [0 0 0 0 0 0 720 4920*k1 0 0 0 0 0 0 -720 -4920*k1];

        A_x(13,9:16) = [1 ,k2 ,k2^2, k2^3, k2^4,k2^5, k2^6, k2^7];
        A_x(14,17:24) = [1 ,k2 ,k2^2, k2^3, k2^4,k2^5, k2^6, k2^7];
        A_x(15,9:24) = [0 1 2*k2 3*k2^2 4*k2^3 5*k2^4 6*k2^5 7*k2^6 0 -1 -2*k2 -3*k2^2 -4*k2^3 -5*k2^4 -6*k2^5 -7*k2^6];
        A_x(16,9:24) = [0 0 2 6*k2 12*k2^2 20*k2^3 30*k2^4 42*k2^5 0 0 -2 -6*k2 -12*k2^2 -20*k2^3 -30*k2^4 -42*k2^5];
        A_x(17,9:24) = [0 0 0 6 24*k2 60*k2^2 120*k2^3 210*k2^4 0 0 0 -6 -24*k2 -60*k2^2 -120*k2^3 -210*k2^4];
        A_x(18,9:24) = [0 0 0 0 24 120*k2 360*k2^2 820*k2^3 0 0 0 0 -24 -120*k2 -360*k2^2 -820*k2^3];
        A_x(19,9:24) = [0 0 0 0 0 120 720*k2 2460*k2^2 0 0 0 0 0 -120 -720*k2 -2460*k2^2];
        A_x(20,9:24) = [0 0 0 0 0 0 720 4920*k2 0 0 0 0 0 0 -720 -4920*k2];

        A_x(21,17:24) = [1 ,k3 ,k3^2, k3^3, k3^4,k3^5, k3^6, k3^7];
        A_x(22,25:32) = [1 ,k3 ,k3^2, k3^3, k3^4,k3^5, k3^6, k3^7];
        A_x(23,17:32) = [0 1 2*k3 3*k3^2 4*k3^3 5*k3^4 6*k3^5 7*k3^6 0 -1 -2*k3 -3*k3^2 -4*k3^3 -5*k3^4 -6*k3^5 -7*k3^6];
        A_x(24,17:32) = [0 0 2 6*k3 12*k3^2 20*k3^3 30*k3^4 42*k3^5 0 0 -2 -6*k3 -12*k3^2 -20*k3^3 -30*k3^4 -42*k3^5];
        A_x(25,17:32) = [0 0 0 6 24*k3 60*k3^2 120*k3^3 210*k3^4 0 0 0 -6 -24*k3 -60*k3^2 -120*k3^3 -210*k3^4];
        A_x(26,17:32) = [0 0 0 0 24 120*k3 360*k3^2 820*k3^3 0 0 0 0 -24 -120*k3 -360*k3^2 -820*k3^3];
        A_x(27,17:32) = [0 0 0 0 0 120 720*k3 2460*k3^2 0 0 0 0 0 -120 -720*k3 -2460*k3^2];
        A_x(28,17:32) = [0 0 0 0 0 0 720 4920*k3 0 0 0 0 0 0 -720 -4920*k3];

        A_x(29,25:32) = [1 ,k4 ,k4^2, k4^3, k4^4,k4^5, k4^6, k4^7];
        A_x(30,25:32) = [0 1 2*k4 3*k4^2 4*k4^3 5*k4^4 6*k4^5 7*k4^6 ];
        A_x(31,25:32) = [0 0 2 6*k4 12*k4^2 20*k4^3 30*k4^4 41*k4^5 ];
        A_x(32,25:32) = [0 0 0 6 24*k4 60*k4^2 120*k4^3 205*k4^4];

        C_x = A_x\y_x;
        C_xx = C_x;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% Y Position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        A_x = zeros(32,32);
        C_x = zeros(32,1);
        y_x = zeros(32,1);
        y_x(1) = waypoints0(1,2);
        y_x(5) = waypoints0(2,2);
        y_x(6) = waypoints0(2,2);
        y_x(13) = waypoints0(3,2);
        y_x(14) = waypoints0(3,2);
        y_x(21) = waypoints0(4,2);
        y_x(22) = waypoints0(4,2);
        y_x(29) = waypoints0(5,2);

        A_x(1,1) = 1;
        A_x(2,2) = 1;
        A_x(3,3) = 2;
        A_x(4,4) = 6;
        A_x(5,1:8) = [1 k1  k1^2 k1^3 k1^4 k1^5 k1^6 k1^7];
        A_x(6,9:16)= [1 k1  k1^2 k1^3 k1^4 k1^5 k1^6 k1^7];
        A_x(7,1:16) = [0 1 2*k1 3*k1^2 4*k1^3 5*k1^4 6*k1^5 7*k1^6 0 -1 -2*k1 -3*k1^2 -4*k1^3 -5*k1^4 -6*k1^5 -7*k1^6];
        A_x(8,1:16) = [0 0 2 6*k1 12*k1^2 20*k1^3 30*k1^4 42*k1^5 0 0 -2 -6*k1 -12*k1^2 -20*k1^3 -30*k1^4 -42*k1^5];
        A_x(9,1:16) = [0 0 0 6 24*k1 60*k1^2 120*k1^3 210*k1^4 0 0 0 -6 -24*k1 -60*k1^2 -120*k1^3 -210*k1^4];
        A_x(10,1:16) = [0 0 0 0 24 120*k1 360*k1^2 820*k1^3 0 0 0 0 -24 -120*k1 -360*k1^2 -820*k1^3];
        A_x(11,1:16) = [0 0 0 0 0 120 720*k1 2460*k1^2 0 0 0 0 0 -120 -720*k1 -2460*k1^2];
        A_x(12,1:16) = [0 0 0 0 0 0 720 4920*k1 0 0 0 0 0 0 -720 -4920*k1];

        A_x(13,9:16) = [1 ,k2 ,k2^2, k2^3, k2^4,k2^5, k2^6, k2^7];
        A_x(14,17:24) = [1 ,k2 ,k2^2, k2^3, k2^4,k2^5, k2^6, k2^7];
        A_x(15,9:24) = [0 1 2*k2 3*k2^2 4*k2^3 5*k2^4 6*k2^5 7*k2^6 0 -1 -2*k2 -3*k2^2 -4*k2^3 -5*k2^4 -6*k2^5 -7*k2^6];
        A_x(16,9:24) = [0 0 2 6*k2 12*k2^2 20*k2^3 30*k2^4 42*k2^5 0 0 -2 -6*k2 -12*k2^2 -20*k2^3 -30*k2^4 -42*k2^5];
        A_x(17,9:24) = [0 0 0 6 24*k2 60*k2^2 120*k2^3 210*k2^4 0 0 0 -6 -24*k2 -60*k2^2 -120*k2^3 -210*k2^4];
        A_x(18,9:24) = [0 0 0 0 24 120*k2 360*k2^2 820*k2^3 0 0 0 0 -24 -120*k2 -360*k2^2 -820*k2^3];
        A_x(19,9:24) = [0 0 0 0 0 120 720*k2 2460*k2^2 0 0 0 0 0 -120 -720*k2 -2460*k2^2];
        A_x(20,9:24) = [0 0 0 0 0 0 720 4920*k2 0 0 0 0 0 0 -720 -4920*k2];

        A_x(21,17:24) = [1 ,k3 ,k3^2, k3^3, k3^4,k3^5, k3^6, k3^7];
        A_x(22,25:32) = [1 ,k3 ,k3^2, k3^3, k3^4,k3^5, k3^6, k3^7];
        A_x(23,17:32) = [0 1 2*k3 3*k3^2 4*k3^3 5*k3^4 6*k3^5 7*k3^6 0 -1 -2*k3 -3*k3^2 -4*k3^3 -5*k3^4 -6*k3^5 -7*k3^6];
        A_x(24,17:32) = [0 0 2 6*k3 12*k3^2 20*k3^3 30*k3^4 42*k3^5 0 0 -2 -6*k3 -12*k3^2 -20*k3^3 -30*k3^4 -42*k3^5];
        A_x(25,17:32) = [0 0 0 6 24*k3 60*k3^2 120*k3^3 210*k3^4 0 0 0 -6 -24*k3 -60*k3^2 -120*k3^3 -210*k3^4];
        A_x(26,17:32) = [0 0 0 0 24 120*k3 360*k3^2 820*k3^3 0 0 0 0 -24 -120*k3 -360*k3^2 -820*k3^3];
        A_x(27,17:32) = [0 0 0 0 0 120 720*k3 2460*k3^2 0 0 0 0 0 -120 -720*k3 -2460*k3^2];
        A_x(28,17:32) = [0 0 0 0 0 0 720 4920*k3 0 0 0 0 0 0 -720 -4920*k3];

        A_x(29,25:32) = [1 ,k4 ,k4^2, k4^3, k4^4,k4^5, k4^6, k4^7];
        A_x(30,25:32) = [0 1 2*k4 3*k4^2 4*k4^3 5*k4^4 6*k4^5 7*k4^6 ];
        A_x(31,25:32) = [0 0 2 6*k4 12*k4^2 20*k4^3 30*k4^4 41*k4^5 ];
        A_x(32,25:32) = [0 0 0 6 24*k4 60*k4^2 120*k4^3 205*k4^4];

       
        C_x = A_x\y_x;
        
        C_xy = C_x;
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% Z Position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        A_x = zeros(32,32);
        C_x = zeros(32,1);
        y_x = zeros(32,1);
        y_x(1) = waypoints0(1,3);
        y_x(5) = waypoints0(2,3);
        y_x(6) = waypoints0(2,3);
        y_x(13) = waypoints0(3,3);
        y_x(14) = waypoints0(3,3);
        y_x(21) = waypoints0(4,3);
        y_x(22) = waypoints0(4,3);
        y_x(29) = waypoints0(5,3);

        A_x(1,1) = 1;
        A_x(2,2) = 1;
        A_x(3,3) = 2;
        A_x(4,4) = 6;
        A_x(5,1:8) = [1 k1  k1^2 k1^3 k1^4 k1^5 k1^6 k1^7];
        A_x(6,9:16)= [1 k1  k1^2 k1^3 k1^4 k1^5 k1^6 k1^7];
        A_x(7,1:16) = [0 1 2*k1 3*k1^2 4*k1^3 5*k1^4 6*k1^5 7*k1^6 0 -1 -2*k1 -3*k1^2 -4*k1^3 -5*k1^4 -6*k1^5 -7*k1^6];
        A_x(8,1:16) = [0 0 2 6*k1 12*k1^2 20*k1^3 30*k1^4 42*k1^5 0 0 -2 -6*k1 -12*k1^2 -20*k1^3 -30*k1^4 -42*k1^5];
        A_x(9,1:16) = [0 0 0 6 24*k1 60*k1^2 120*k1^3 210*k1^4 0 0 0 -6 -24*k1 -60*k1^2 -120*k1^3 -210*k1^4];
        A_x(10,1:16) = [0 0 0 0 24 120*k1 360*k1^2 820*k1^3 0 0 0 0 -24 -120*k1 -360*k1^2 -820*k1^3];
        A_x(11,1:16) = [0 0 0 0 0 120 720*k1 2460*k1^2 0 0 0 0 0 -120 -720*k1 -2460*k1^2];
        A_x(12,1:16) = [0 0 0 0 0 0 720 4920*k1 0 0 0 0 0 0 -720 -4920*k1];

        A_x(13,9:16) = [1 ,k2 ,k2^2, k2^3, k2^4,k2^5, k2^6, k2^7];
        A_x(14,17:24) = [1 ,k2 ,k2^2, k2^3, k2^4,k2^5, k2^6, k2^7];
        A_x(15,9:24) = [0 1 2*k2 3*k2^2 4*k2^3 5*k2^4 6*k2^5 7*k2^6 0 -1 -2*k2 -3*k2^2 -4*k2^3 -5*k2^4 -6*k2^5 -7*k2^6];
        A_x(16,9:24) = [0 0 2 6*k2 12*k2^2 20*k2^3 30*k2^4 42*k2^5 0 0 -2 -6*k2 -12*k2^2 -20*k2^3 -30*k2^4 -42*k2^5];
        A_x(17,9:24) = [0 0 0 6 24*k2 60*k2^2 120*k2^3 210*k2^4 0 0 0 -6 -24*k2 -60*k2^2 -120*k2^3 -210*k2^4];
        A_x(18,9:24) = [0 0 0 0 24 120*k2 360*k2^2 820*k2^3 0 0 0 0 -24 -120*k2 -360*k2^2 -820*k2^3];
        A_x(19,9:24) = [0 0 0 0 0 120 720*k2 2460*k2^2 0 0 0 0 0 -120 -720*k2 -2460*k2^2];
        A_x(20,9:24) = [0 0 0 0 0 0 720 4920*k2 0 0 0 0 0 0 -720 -4920*k2];

        A_x(21,17:24) = [1 ,k3 ,k3^2, k3^3, k3^4,k3^5, k3^6, k3^7];
        A_x(22,25:32) = [1 ,k3 ,k3^2, k3^3, k3^4,k3^5, k3^6, k3^7];
        A_x(23,17:32) = [0 1 2*k3 3*k3^2 4*k3^3 5*k3^4 6*k3^5 7*k3^6 0 -1 -2*k3 -3*k3^2 -4*k3^3 -5*k3^4 -6*k3^5 -7*k3^6];
        A_x(24,17:32) = [0 0 2 6*k3 12*k3^2 20*k3^3 30*k3^4 42*k3^5 0 0 -2 -6*k3 -12*k3^2 -20*k3^3 -30*k3^4 -42*k3^5];
        A_x(25,17:32) = [0 0 0 6 24*k3 60*k3^2 120*k3^3 210*k3^4 0 0 0 -6 -24*k3 -60*k3^2 -120*k3^3 -210*k3^4];
        A_x(26,17:32) = [0 0 0 0 24 120*k3 360*k3^2 820*k3^3 0 0 0 0 -24 -120*k3 -360*k3^2 -820*k3^3];
        A_x(27,17:32) = [0 0 0 0 0 120 720*k3 2460*k3^2 0 0 0 0 0 -120 -720*k3 -2460*k3^2];
        A_x(28,17:32) = [0 0 0 0 0 0 720 4920*k3 0 0 0 0 0 0 -720 -4920*k3];

        A_x(29,25:32) = [1 ,k4 ,k4^2, k4^3, k4^4,k4^5, k4^6, k4^7];
        A_x(30,25:32) = [0 1 2*k4 3*k4^2 4*k4^3 5*k4^4 6*k4^5 7*k4^6 ];
        A_x(31,25:32) = [0 0 2 6*k4 12*k4^2 20*k4^3 30*k4^4 41*k4^5 ];
        A_x(32,25:32) = [0 0 0 6 24*k4 60*k4^2 120*k4^3 205*k4^4];
        
        
        C_x = A_x\y_x;
        
        C_xz = C_x;

     
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
 
    
    
else
    
    if(t > traj_time(end))
        t = traj_time(end) - 0.0001;
    end
%     
%     t_index = find(traj_time >= t,1) - 1;
%     t_index = max(t_index, 1);
%     
%     if t == 0
%         t_index = 1;
%     end
    
   
   desired_state.pos = zeros(3,1);
   desired_state.vel = zeros(3,1);
   desired_state.acc = zeros(3,1);
   
   scale = t;
    

   
    
    if t < traj_time(2)
        
        %scale = (t - traj_time(1))/d0(1);
        
        
        desired_state.pos(1) = C_xx(1) + C_xx(2)*scale + C_xx(3)*scale^2 + C_xx(4)*scale^3 + C_xx(5)*scale^4 +C_xx(6)*scale^5 +C_xx(7)*scale^6 + C_xx(8)*scale^7 ;
        desired_state.vel(1) = C_xx(2) + 2*C_xx(3)*scale + 3*C_xx(4)*scale^2 + 4*C_xx(5)*scale^3 + 5*C_xx(6)*scale^4 +  6*C_xx(7)*scale^5  +  7*C_xx(8)*scale^6 ;
        desired_state.acc(1) =  2*C_xx(3) + 6*C_xx(4)*scale + 12*C_xx(5)*scale^2 + 20*C_xx(6)*scale^3 +  30*C_xx(7)*scale^4  +  42*C_xx(8)*scale^5 ;
        
        desired_state.pos(2) = C_xy(1) + C_xy(2)*scale + C_xy(3)*scale^2 + C_xy(4)*scale^3 + C_xy(5)*scale^4 +C_xy(6)*scale^5 +C_xy(7)*scale^6 +C_xy(8)*scale^7 ;
        desired_state.vel(2) = C_xy(2) + 2*C_xy(3)*scale + 3*C_xy(4)*scale^2 + 4*C_xy(5)*scale^3 + 5*C_xy(6)*scale^4 +  6*C_xy(7)*scale^5  +  7*C_xy(8)*scale^6 ;
        desired_state.acc(2) =  2*C_xy(3) + 6*C_xy(4)*scale + 12*C_xy(5)*scale^2 + 20*C_xy(6)*scale^3 +  30*C_xy(7)*scale^4  +  42*C_xy(8)*scale^5 ;
        
        desired_state.pos(3) = C_xz(1) + C_xz(2)*scale + C_xz(3)*scale^2 + C_xz(4)*scale^3 + C_xz(5)*scale^4 +C_xz(6)*scale^5 + C_xz(7)*scale^6 + C_xz(8)*scale^7 ;
        desired_state.vel(3) = C_xz(2) + 2*C_xz(3)*scale + 3*C_xz(4)*scale^2 + 4*C_xz(5)*scale^3 + 5*C_xz(6)*scale^4 +  6*C_xz(7)*scale^5  +  7*C_xz(8)*scale^6 ;
        desired_state.acc(3) =  2*C_xz(3) + 6*C_xz(4)*scale + 12*C_xz(5)*scale^2 + 20*C_xz(6)*scale^3 +  30*C_xz(7)*scale^4  +  42*C_xz(8)*scale^5 ;
        
        
    elseif t >= traj_time(2) &&  t < traj_time(3)
        
        %scale = (t - traj_time(1))/d0(2);
        
        desired_state.pos(1) = C_xx(9) + C_xx(10)*scale + C_xx(11)*scale^2 + C_xx(12)*scale^3 + C_xx(13)*scale^4 +C_xx(14)*scale^5 +C_xx(15)*scale^6 + C_xx(16)*scale^7 ;
        desired_state.vel(1) = C_xx(10) + 2*C_xx(11)*scale + 3*C_xx(12)*scale^2 + 4*C_xx(13)*scale^3 + 5*C_xx(14)*scale^4 +  6*C_xx(15)*scale^5  +  7*C_xx(16)*scale^6 ;
        desired_state.acc(1) =  2*C_xx(11) + 6*C_xx(12)*scale + 12*C_xx(13)*scale^2 + 20*C_xx(14)*scale^3 +  30*C_xx(15)*scale^4  +  42*C_xx(16)*scale^5 ;
        
        desired_state.pos(2) = C_xy(9) + C_xy(10)*scale + C_xy(11)*scale^2 + C_xy(12)*scale^3 + C_xy(13)*scale^4 +C_xy(14)*scale^5 +C_xy(15)*scale^6 +C_xy(16)*scale^7 ;
        desired_state.vel(2) = C_xy(10) + 2*C_xy(11)*scale + 3*C_xy(12)*scale^2 + 4*C_xy(13)*scale^3 + 5*C_xy(14)*scale^4 +  6*C_xy(15)*scale^5  +  7*C_xy(16)*scale^6 ;
        desired_state.acc(2) =  2*C_xy(11) + 6*C_xy(12)*scale + 12*C_xy(13)*scale^2 + 20*C_xy(14)*scale^3 +  30*C_xy(15)*scale^4  +  42*C_xy(16)*scale^5 ;
        
        desired_state.pos(3) = C_xz(9) + C_xz(10)*scale + C_xz(11)*scale^2 + C_xz(12)*scale^3 + C_xz(13)*scale^4 +C_xz(14)*scale^5 + C_xz(15)*scale^6 + C_xz(16)*scale^7 ;
        desired_state.vel(3) = C_xz(10) + 2*C_xz(11)*scale + 3*C_xz(12)*scale^2 + 4*C_xz(13)*scale^3 + 5*C_xz(14)*scale^4 +  6*C_xz(15)*scale^5  +  7*C_xz(16)*scale^6 ;
        desired_state.acc(3) =  2*C_xz(11) + 6*C_xz(12)*scale + 12*C_xz(13)*scale^2 + 20*C_xz(14)*scale^3 +  30*C_xz(15)*scale^4  +  42*C_xz(16)*scale^5 ;
        
    elseif t >= traj_time(3) &&  t < traj_time(4)
        
        %scale = (t - traj_time(2))/d0(3);
        
        
        desired_state.pos(1) = C_xx(17) + C_xx(18)*scale + C_xx(19)*scale^2 + C_xx(20)*scale^3 + C_xx(21)*scale^4 +C_xx(22)*scale^5 +C_xx(23)*scale^6 + C_xx(24)*scale^7 ;
        desired_state.vel(1) = C_xx(18) + 2*C_xx(19)*scale + 3*C_xx(20)*scale^2 + 4*C_xx(21)*scale^3 + 5*C_xx(22)*scale^4 +  6*C_xx(23)*scale^5  +  7*C_xx(24)*scale^6 ;
        desired_state.acc(1) =  2*C_xx(19) + 6*C_xx(20)*scale + 12*C_xx(21)*scale^2 + 20*C_xx(22)*scale^3 +  30*C_xx(23)*scale^4  +  42*C_xx(24)*scale^5 ;
        
        desired_state.pos(2) = C_xy(17) + C_xy(18)*scale + C_xy(19)*scale^2 + C_xy(20)*scale^3 + C_xy(21)*scale^4 +C_xy(22)*scale^5 +C_xy(23)*scale^6 +C_xy(24)*scale^7 ;
        desired_state.vel(2) = C_xy(18) + 2*C_xy(19)*scale + 3*C_xy(20)*scale^2 + 4*C_xy(21)*scale^3 + 5*C_xy(22)*scale^4 +  6*C_xy(23)*scale^5  +  7*C_xy(24)*scale^6 ;
        desired_state.acc(2) =  2*C_xy(19) + 6*C_xy(20)*scale + 12*C_xy(21)*scale^2 + 20*C_xy(22)*scale^3 +  30*C_xy(23)*scale^4  +  42*C_xy(24)*scale^5 ;
        
        desired_state.pos(3) = C_xz(17) + C_xz(18)*scale + C_xz(19)*scale^2 + C_xz(20)*scale^3 + C_xz(21)*scale^4 +C_xz(22)*scale^5 + C_xz(23)*scale^6 + C_xz(24)*scale^7 ;
        desired_state.vel(3) = C_xz(18) + 2*C_xz(19)*scale + 3*C_xz(20)*scale^2 + 4*C_xz(21)*scale^3 + 5*C_xz(22)*scale^4 +  6*C_xz(23)*scale^5  +  7*C_xz(24)*scale^6 ;
        desired_state.acc(3) =  2*C_xz(19) + 6*C_xz(20)*scale + 12*C_xz(21)*scale^2 + 20*C_xz(22)*scale^3 +  30*C_xz(23)*scale^4  +  42*C_xz(24)*scale^5 ;
        
    elseif t >= traj_time(4) && t < traj_time(5)
        
        %scale = (t - traj_time(3))/d0(4);
        
        desired_state.pos(1) = C_xx(25) + C_xx(26)*scale + C_xx(27)*scale^2 + C_xx(28)*scale^3 + C_xx(29)*scale^4 + C_xx(30)*scale^5 + C_xx(31)*scale^6 + C_xx(32)*scale^7 ;
        desired_state.vel(1) = C_xx(26) + 2*C_xx(27)*scale + 3*C_xx(28)*scale^2 + 4*C_xx(29)*scale^3 + 5*C_xx(30)*scale^4 +  6*C_xx(31)*scale^5  +  7*C_xx(32)*scale^6 ;
        desired_state.acc(1) =  2*C_xx(27) + 6*C_xx(28)*scale + 12*C_xx(29)*scale^2 + 20*C_xx(30)*scale^3 +  30*C_xx(31)*scale^4  +  42*C_xx(32)*scale^5 ;
        
        desired_state.pos(2) = C_xy(25) + C_xy(26)*scale + C_xy(27)*scale^2 + C_xy(28)*scale^3 + C_xy(29)*scale^4 +C_xy(30)*scale^5 +C_xy(31)*scale^6 + C_xy(32)*scale^7 ;
        desired_state.vel(2) = C_xy(26) + 2*C_xy(27)*scale + 3*C_xy(28)*scale^2 + 4*C_xy(29)*scale^3 + 5*C_xy(30)*scale^4 +  6*C_xy(31)*scale^5  +  7*C_xy(32)*scale^6 ;
        desired_state.acc(2) =  2*C_xy(27) + 6*C_xy(28)*scale + 12*C_xy(29)*scale^2 + 20*C_xy(30)*scale^3 +  30*C_xy(31)*scale^4  +  42*C_xy(32)*scale^5 ;
        
        desired_state.pos(3) = C_xz(25) + C_xz(26)*scale + C_xz(27)*scale^2 + C_xz(28)*scale^3 + C_xz(29)*scale^4 +C_xz(30)*scale^5 + C_xz(31)*scale^6 + C_xz(32)*scale^7 ;
        desired_state.vel(3) = C_xz(26) + 2*C_xz(27)*scale + 3*C_xz(28)*scale^2 + 4*C_xz(29)*scale^3 + 5*C_xz(30)*scale^4 +  6*C_xz(31)*scale^5  +  7*C_xz(32)*scale^6 ;
        desired_state.acc(3) =  2*C_xz(27) + 6*C_xz(28)*scale + 12*C_xz(29)*scale^2 + 20*C_xz(30)*scale^3 +  30*C_xz(31)*scale^4  +  42*C_xz(32)*scale^5 ;
         
    elseif t >= traj_time(5)
        desired_state.pose = waypoints0(5,:);
        %desired_state.vel = zeros(3,1);
        %desired_state.acc = zeros(3,1);

    end
    
   %disp(desired_state.pos);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
        
        
end
%


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;


















end

