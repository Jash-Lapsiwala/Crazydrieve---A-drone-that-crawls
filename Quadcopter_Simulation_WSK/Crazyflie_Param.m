% Crazyflie parameters
g = 9.80665;

l1 = 4.445; % cm  1/2 of length of body
l2 = 4.5;

kq = 0.006;

% Rotor/Propeller parameters
m = 0.00036; % new propeller mass kg
J = 1/12*m*(0.051^2+0.005^2); % moment of inertia from mass, kgm^2
b = 3.5077E-10; % motor viscous friction constant Nms


%drone parameters
m_drone = 0.04316;   % kg, without propellers
% total weight of the drone = 45g
% Ix_drone = 1.112951*10^-5;% kg*m^2
Ix_drone = 2.934e-5;
% Iy_drone = 1.1143608*10^-5;  % kg*m^2
Iy_drone = 4.981e-5;
% Iz_drone = 2.162056*10^-5;  % kg*m^2
Iz_drone = 7.646e-5;
b_drone = 1*10^-9; % kg*m^2 drone's x,y,z translational drag coefficient


%% =====================================================================
%  HOVER LINEARIZATION
% ======================================================================

m_prop = m;
m_tot = m_drone + 4*m_prop;

d_arm = 0.04475;
d_perp = d_arm/sqrt(2);

Ixx = Ix_drone + 4*m_prop*d_perp^2;
Iyy = Iy_drone + 4*m_prop*d_perp^2;
Izz = Iz_drone + 4*m_prop*d_arm^2;

rho = 1.225;
D_prop = 0.051;
kT = 0.2025;

C_T = kT*rho*D_prop^4/3600;
C_Q = kq*C_T;

% x_e = 0; 
% y_e = 0; 
% z_e = 1.25; 
% psi_e = 0;
% X_e = [x_e y_e z_e psi_e 0 0 0 0 0 0 0 0]';

T_e = m_tot*g/4;
omega_e = sqrt(T_e/C_T);

U_e = omega_e*ones(4,1);

A = zeros(12);
A(1,7) = 1;      % xdot = u
A(2,8) = 1;      % ydot = v
A(3,9) = 1;      % zdot = w
A(4,10) = 1;     % psidot = r
A(5,11) = 1;     % thetadot = q
A(6,12) = 1;     % phidot = p
A(7,5) =  g;     % udot =  g*theta
A(8,6) = -g;     % vdot = -g*phi

B = zeros(12, 4);
B(9,:) = omega_e * (2*C_T/m_tot) * [ 1  1  1  1];              % wdot
B(10,:) = omega_e * (2*C_Q/Izz) * [-1  1 -1  1];               % rdot
B(11,:) = omega_e * (sqrt(2)*d_arm*C_T/Iyy) * [-1  1  1 -1];   % qdot
B(12,:) = omega_e * (sqrt(2)*d_arm*C_T/Ixx) * [-1 -1  1  1];   % pdot

C = eye(12);
D = zeros(12, 4);

B_v = zeros(12, 4);
B_v(9,1) = 1/m_tot;    % wdot = dT/m
B_v(10,4) = 1/Izz;     % rdot = tau_z/Izz
B_v(11,3) = 1/Iyy;     % qdot = tau_y/Iyy
B_v(12,2) = 1/Ixx;     % pdot = tau_x/Ixx

Mix = [ 1, 1, 1, 1;
    -d_perp, -d_perp,  d_perp,  d_perp;
    -d_perp,  d_perp,  d_perp, -d_perp;
    kq,     -kq,      kq,     -kq];  
invMix = inv(Mix);  

dFdw = 2*C_T*omega_e;
B_F  = B/dFdw;

S_thrust = 2.15*0.12;                % N per unit normalized force (from flight logs)
f_hover  = 0.428; 

pwm2rpm_a = 0.2685;   pwm2rpm_b = 4070.3;
rpm2pwm   = @(rpm) (rpm - pwm2rpm_b)/pwm2rpm_a;
pwm_e     = rpm2pwm(omega_e);

Co = ctrb(A,B);
fprintf('m_tot = %.4f kg\n', m_tot);
fprintf('Ixx = %.3e kg m^2, Iyy = %.3e kg m^2, Izz = %.3e kg m^2\n', Ixx, Iyy, Izz);
fprintf('C_T = %.3e N/RPM^2, C_Q = %.3e Nm/RPM^2\n', C_T, C_Q);
fprintf('omega_e = %.0f RPM, pwm_e = %.0f\n', omega_e, pwm_e);
fprintf('rank(ctrb) = %d of %d\n', rank(Co), 12);

%% =====================================================================
%  CONTROLLER (LQR)
%  =====================================================================

Q =  diag ([20000, 20000, 32000, 16000, 68000, 68000, 4000, 4000, 10, 4000, 4000, 4000]);
% R =  0.00003*eye(4);
R = diag([4.84e5, 4.83e8, 4.83e8, 1.34e10]);
% R = eye(4);
% R = diag([1/0.15^2, 1/0.006^2, 1/0.006^2, 1/0.0012^2]);
K = lqr(A, B_v, Q, R);
writematrix(K, 'K.csv');
disp(K)
% disp(eig(A-B_v*K))
% fprintf('K = %d\n', K);