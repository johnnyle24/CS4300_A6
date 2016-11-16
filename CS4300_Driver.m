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
clc;
x0 = 0;
y0 = 0;
vx0 = 4;
vy0 = 4;
theta = pi/8;
del_t = 0.05;
max_time = 20;
t_max = 70;
g = -9.8;
r = 0.001;
q = 1;
obs_freq = 0.35;

%Linear motion
%[xt,at,zt,Sigma2_trace] = CS4300_A5_driver_lin(x0,y0,vx0,vy0,max_time,del_t,theta);

%Projectile motion
[xt,at,zt,St] = CS4300_driver_proj(del_t,t_max,obs_freq,g,r,q);

%  hold on
%  axval = at(:,1);
%  ayval = at(:,2);
%  zxval = zt(:,1);
%  zyval = zt(:,2);
%  xval = xt(:,1);
%  yval = xt(:,2);
%  
%  plot(xval,yval,'.');
%  plot(zxval,zyval,'o');
%  plot(axval,ayval);
% 
% legend('estimate','observed','actual');

hold on
evx = xt(:,3);
evy = xt(:,4);
avx = at(:,3);
avy = at(:,4);
% 
% % 
plot(evx,'x');
plot(evy,'red');
plot(avx,'o');
plot(avy,'*');
% % 
legend('estimated vel x','estimated vel y','actual vel x', 'actual vel y');
hold off

disp('DONE');
