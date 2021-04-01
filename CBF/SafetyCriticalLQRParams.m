%% Plant
% Double integrator point-mass plant model
% The states are position and velocity.

% Dimentions
Nx = 2;

% State-space matrices
A = [0 1;0 0];
B = [0; 1];
C = eye(Nx);
D = [0;0];
G = ss(A,B,C,D);

% Initial conditions
x0 = [10;10];

% Drive the states to origin
Ref = [0;0];

% Simulation time
tf_upperbound = 50;

%% Obstacle location
cT = [10;-5];
rT = 4;

%% Nominal Control Design
% Design LQR to stabilize the plant
R = 5;
Q = 0.5*eye(Nx);
[K,P] = lqr(A,B,Q,R);

% Nominal LQR
filterCase = 1;

% Default value for safety filter design parameter
eta = 2; 