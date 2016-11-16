function [mu_t,Sigma_t] = CS4300_KF(mu_tm1,Sigma_tm1,u_t,z_t,A_t,R_t,...
B_t,C_t,Q_t)
% CS4300_KF - one step in Kalman Filter
% On input:
%     mu_tm1 (nx1 vector): previous state estimate vector
%     Sigma_tm1 (nxn matrix): state covariance matrix
%     u_t (nx1 vector): control vector
%     z_t (nx1 vector): measurement vector
%     A_t (nxn matrix): state transition matrix
%     R_t (nxn matrix): statre transition covariance matrix
%     C_t (nxn matrix): linear transform for measurement equation
%     Q_t (nxn matrix): noise covariance matrix
% On output:
%     mu_t (nx1 vector): next state estimate
%     Sigma_t (nxn matrix): state covariance matrix
% Call:
%     [x,Sigma2] = CS4300_KF(x,Sigma2,u,z,A,B,R,C,Q);
% Author:
%     Johnny Le and Trung Le
%     UU
%     Fall 2016
%

mu_t = (A_t * mu_tm1) + (B_t * u_t);
Sigma_t = A_t * Sigma_tm1 * A_t' + R_t;

K_t = Sigma_t * C_t' * inv((C_t * Sigma_t * C_t' + Q_t));
mu_t = mu_t + K_t * (z_t - C_t*mu_t);
I = eye(4);
Sigma_t = (I-K_t * C_t) * Sigma_t;




