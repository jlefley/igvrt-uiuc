%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roomba_sim
%
% inputs
%   vr: vector of right motor velocities. dimension should be (N,1) [m/s]
%   vl: vector of left motor velocities. dimension should be (N,1) [m/s]
%   dt: timestep [s]
%   N: number of timesteps to simulate
%   motor_std: std. dev. of measurement error [m/s]
%
% outputs
%   t: sim time [s]
%   x: estimated x coord (without kalman filter) [m]
%   y: estimated y coord (without kalman filter) [m]
%   ang: estimated angle (without kalman filter) [rad]
%   kf_x: estimated x coord (with kalman filter) [m]
%   kf_y: estimated y coord (with kalman filter) [m]
%   kf_ang: estimated angle (with kalman filter) [rad]
%
% example
%   vr = 0.5*ones(1000,1);
%   vr(500:1000) = -0.5;
%   vl = vr;
%   [t x y ang kf_x kf_y kf_ang] = roomba_sim(vr,vl,0.1,1000,0.1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t x y ang kf_x kf_y kf_ang] = roomba_sim(vr,vl,dt,N,motor_std)

b = 0.258; % wheel axle length [m]

t = zeros(N,1);
x = zeros(N,1);
y = zeros(N,1);
ang = zeros(N,1);
kf_x = zeros(N,1);
kf_y = zeros(N,1);
kf_ang = zeros(N,1);

% set up xy kalman structure
s_xy.x = zeros(4,1);
s_xy.x(3) = vr(1);
s_xy.x(4) = vl(1);
s_xy.P = zeros(4,4);
s_xy.z = zeros(4,1);
s_xy.u = zeros(4,1);
s_xy.u(3) = vr(1);
s_xy.u(4) = vl(1);
s_xy.A = eye(4,4);
s_xy.B = eye(4,4);
s_xy.Q = eye(4,4)*0.0000005;
s_xy.R = eye(4,4)*0.5;
s_xy.H = eye(4,4);
s_xy.H(1,1) = 0;
s_xy.H(2,2) = 0;

% set up ang kalman structure
s_ang.x = zeros(4,1);
s_ang.x(3) = vr(1);
s_ang.x(4) = vl(1);
s_ang.P = zeros(4,4);
s_ang.z = zeros(4,1);
s_ang.u = zeros(4,1);
s_ang.u(3) = vr(1);
s_ang.u(4) = vl(1);
s_ang.A = eye(4,4);
s_ang.B = eye(4,4);
s_ang.Q = eye(4,4)*0.0000005;
s_ang.R = eye(4,4)*0.5;
s_ang.H = eye(4,4);
s_ang.H(1,1) = 0;
s_ang.H(2,2) = 0;

t(1) = 0;
for ind=2:N

    t(ind) = t(ind - 1) + dt;
    
    [vr_meas vl_meas] = meas(vr(ind),vl(ind),motor_std);
    
    % non-kalman estimation
    ang(ind) = ang(ind - 1) + (((vr_meas - vl_meas)*dt)/b);
    
    x(ind) = x(ind - 1) + (0.5*dt*vr_meas*cos(ang(ind))) + (0.5*dt*vl_meas*cos(ang(ind)));
    y(ind) = y(ind - 1) + (0.5*dt*vr_meas*sin(ang(ind))) + (0.5*dt*vl_meas*sin(ang(ind)));
    
    % kalman estimation
    
    % kalman-ang
    s_ang.A = eye(4,4);
    s_ang.A(1,3) = (dt/b);
    s_ang.A(1,4) = (-1)*(dt/b);
    s_ang.A(2,3) = (1/b);
    s_ang.A(2,4) = (-1)*(1/b);
    s_ang.A(2,2) = 0;
    
    s_ang.z = [0 0 vr_meas vl_meas]';
    s_ang.u = ([0 0 vr(ind) vl(ind)]') - ([0 0 s_ang.x(3) s_ang.x(4)]');
    
    kf_ang(ind) = s_ang.x(1);
    
    s_ang = kalmanf(s_ang);
    
    % kalman-xy
    theta = s_ang.x(1);
    
    s_xy.A = eye(4,4);
    s_xy.A(1,3) = 0.5*dt*cos(theta);
    s_xy.A(1,4) = 0.5*dt*cos(theta);
    s_xy.A(2,3) = 0.5*dt*sin(theta);
    s_xy.A(2,4) = 0.5*dt*sin(theta);
    
    s_xy.z = [0 0 vr_meas vl_meas]';
    s_xy.u = ([0 0 vr(ind) vl(ind)]') - ([0 0 s_xy.x(3) s_xy.x(4)]');
    
    kf_x(ind) = s_xy.x(1);
    kf_y(ind) = s_xy.x(2);
    
    s_xy = kalmanf(s_xy);
    
end

xlabel('t [s]');
hold on;
ylabel('x [m]');
plot(t,x,'b');
plot(t,kf_x,'r');
plot(t,vr,'k');
plot(t,vl,'k');
