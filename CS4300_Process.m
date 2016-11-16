function x = CS4300_Process(xa,A,B,U,R)
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
%     xa = CS4300_Process(xa,A,B,U,R);
% Author:
%     Johnny Le and Trung Le
%     UU
%     Fall 2016
%

x = (A * xa) + (B * U) + sqrt(R) * randn(4,1);

