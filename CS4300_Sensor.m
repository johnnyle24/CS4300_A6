function z = CS4300_Sensor(c,x,q)
% CS4300_fall_sensor - observes falling object
% On input:
%     y (float): actual height of object
%     C (float): sensor process model
%     sigma2 (float): variance of Gaussian noise for sensor
% On output:
%     z (float): sensed height of object
% Call:
%     z = CS4300_fall_sensor(y,1,0.1);
% Author:
%     T. Henderson
%     UU
%     Fall 2016
%

z = c * x + sqrt(q) + randn(2,1);
