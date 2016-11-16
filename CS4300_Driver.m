function p = CS4300_A6_Driver()
% CS4300_Gaussian_1D - 1D Gaussian distribution
% On input:
%     x (float): a real number
%     mu (float): mean of distribution
%     sigma2 (float): variance of distribution
% On output:
%     p (float): probability of sampling value x frm distribution
% Call:
%     p = CS4300_Gaussian_1D(0.3,0,1);
% Author:
%     T. Henderson
%     UU
%     Fall 2016
%

%Linear motion
x0 = 0;
y0 = 0;
vx0 = 4;
vy0 = 4;
theta = pi/8;
del_t = 0.05;
max_time = 20;
t_max = 70;
g = -9.8;
r = 2;
q = 5;

[x_trace,a_trace,z_trace,Sigma2_trace] = CS4300_A5_driver_lin(x0,y0,vx0,vy0,max_time,del_t,theta);
hold on
axval = a_trace(:,1);
ayval = a_trace(:,2);
zxval = z_trace(:,1);
zyval = z_trace(:,2);
xval = x_trace(:,1);
yval = x_trace(:,2);

plot(xval,yval);
plot(zxval,zyval,'o');
plot(axval,ayval);


%Projectile motion
%[xt,at,zt,St] = CS4300_driver_proj(del_t,t_max,obs_freq,g,r,q);

disp('DONE');
