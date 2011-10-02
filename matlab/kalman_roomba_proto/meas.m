%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% meas
%
% inputs
%   vr: specified right motor velocity [m/s]
%   vl: specified left motor velocity [m/s]
%   motor_std: std. dev. of measurement error [m/s]
%
% outputs
%   vr_meas: measured right motor velocity [m/s]
%   vl_meas: measured left motor velocity [m/s]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vr_meas vl_meas] = meas(vr,vl,motor_std)

vr_meas = vr + (randn*motor_std);
vl_meas = vl + (randn*motor_std);

