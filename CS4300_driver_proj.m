function [xt,at,zt,st] = CS4300_driver_proj(del_t,t_max,obs_freq,g,r,q)
% CS4300_driver_proj - driver for Kalman Filter projectile tracking
% This script demonstrates the use of the kalman filter in projectile
% tracking.
%
% A simple model for a 2D projectile is obtained and used for the
% kalman
% filter transition model. The actual trajectory of the projectile is
% simulated using a more complex and accurate model. Observations of
% the
% actual projectile trajectory are obtained. The kalman filter is used
% to
% generate beliefs on the actual projectile state. All three
% trajectories
% (the simple model, the complex model, and the kalman belief) are
% plotted,
% along with the observations made.
% On input:
%     dt (float): simulation time step in seconds (e.g., 0.1)
%     t_max (float): max simulation time in seconds (e.g., 35)
%     obs_freq (float): frequency with which time steps are observed,
%        range [0,1] (e.g., .35)
%     g (float): gravitational acceleration in m/sˆ2 (9.8)
%     r (float): transition noise; (e.g., 2)
%           increase for faster response to observations
%     q (float):observation noise; increase for greater
%           observation spread (e.g., 10)
% On output:
%     xt (nx4 array): trace of estimated pose
%     at (nx4 array): trace of actual pose
%     zt (mx2 array): trace of sensor data
%     St (struct array): covariance matrix trace
%       (i).Sigma2 (4x4 array): estimated vector covariance
% Call:
%     [xt,at,zt,St] = CS4300_driver_proj(0.1,35,0.35,9.8,2,10);
% Author:
%     Johnny Le and Trung Le
%     UU
%     Fall 2016
%

A = [1,0,del_t,0;0,1,0,del_t;0,0,1,0;0,0,0,1];
B = [(del_t * del_t) / 2, 0; 0, (del_t * del_t) / 2; del_t, 0; 0, del_t];
u = [0;g];
R = eye(4,4) * r;
C = [1,0,0,0;0,1,0,0];
Q = eye(2,2) * q;

%x actual
xa = [0;0;100;100];
x_ideal = xa;
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

t_vals = [del_t:del_t:t_max];
num_steps = length(t_vals);

% for s = 1:num_steps
%    xa = CS4300_Process(xa,A,B,U,R);
%    at = [at;xa'];
%    z = CS4300_fall_sensor(c,xa,Q);
%    zt = [zt:z'];
%    [x,sigma] = CS4300_KF(xt(end), st(end), sigma,u,zt,A,R,B,C,Q);
%    xt = [xt;x'];
%    St(end+1).sigma2 = sigma;
% end


for s = 1:num_steps
   if(xa(2) < 0)
      break; 
   end
   x_ideal = CS4300_Process(x_ideal,A,B,u,R);
   
   x_noise = [sqrt(R(1,1)) *randn; sqrt(R(2,2))*randn; sqrt(R(3,3))*randn; sqrt(R(4,4))*randn];
   
   xa = x_ideal + x_noise;
  
   z = CS4300_Sensor(C,xa,Q);

   [x,sigma] = CS4300_KF(x, st(end).sigma2,u,z,A,R,B,C,Q);
   
   
   if(mod(s,obs_freq) == 0) 
        at = [at;xa'];
        xt = [xt;x'];
        st(end+1).sigma2 = sigma;
        zt = [zt;z'];  
   end
end

disp('done2')
