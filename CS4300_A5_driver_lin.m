function [x_trace,a_trace,z_trace,Sigma2_trace] = CS4300_A5_driver_lin(...
x0,y0,vx0,vy0,max_time,del_t,theta)
% CS4300_A5_driver_lin - driver function for linear Kalman Filter
% On input:
%     x0 (float): initial x location
%     y0 (float): initial y location
%     vx0 (float): initial x velocity
%     vy0 (float): initial y velocity
%     max_time (float): max time for tracking
%     del_t (float): time step size
%     theta (float): angle of line (in radians)
% On output:
%     x_trace (nx2 array): each row has estimated pose (x,y vals)
%     a_trace (nx2 array): actual location at each time step
%     z_trace (nx2 array): sensed location at each time step
%     Sigma2_trace (struct array): covariance of estimated location
%       (i).Sigma2 (2x2 array): covariance matrix for i_th step
% Call:
%     [xt,at,zt,St] = CS4300_A5_driver_lin(0,0,1,1,1,0.1,pi/4);
% Author:
%     Johnny Le and Trung Le
%     UU
%     Fall 2016
%

%Initialize matrices
A = [1,0,del_t,0;0,1,0,del_t;0,0,1,0;0,0,0,1];
B = [(del_t * del_t) / 2, 0; 0, (del_t * del_t) / 2; del_t, 0; 0, del_t];
u = [0;0];
R = [0.001,0,0,0;0,0.001,0,0;0,0,0.001,0;0,0,0,0.001];
C = [1,0,0,0;0,1,0,0];
Q = [0.001,0;0,0.001];

%x actual
xa = [x0;y0;vx0;vy0];
at = xa';

%z actual
z = CS4300_Sensor(C,xa,Q);
zt = z';

%x trace
x = [z(1);z(2);0;0];
xt = x';

%sigma trace
sigma = R;
st(1).sigma2 = sigma;

R = eye(4) * 0.001;
Q = eye(2) * 0.5;

t_vals = [del_t:del_t:max_time];
num_steps = length(t_vals);

for s = 1:num_steps
   x = CS4300_Process(xa,A,B,u,R);
   
   x_noise = [sqrt(R(1,1)) *randn; sqrt(R(2,2))*randn; sqrt(R(3,3))*randn; sqrt(R(4,4))*randn];
   
   xa = x + x_noise;
     
   at = [at;xa'];
   
   z = CS4300_Sensor(C,xa,Q);
   zt = [zt;z'];
   [x,sigma] = CS4300_KF(xa, st(end).sigma2,u,z,A,R,B,C,Q);
   xt = [xt;x'];
   st(end+1).sigma2 = sigma;
end

x_trace = xt(: , 1:2);
Sigma2_trace = st;
z_trace = zt;
a_trace = at;






