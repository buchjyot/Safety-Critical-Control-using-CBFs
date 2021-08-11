%% Plant

% State-space matrices
A = 0;
B = 1;
C = 1;
D = 0;
G = ss(A,B,C,D);

% Initial conditions
x0 = 0.5;

% Drive the states to origin
Ref = 0;

% Simulation time
tf_upperbound = 50;

%% Nominal Control Design
% Design LQR to stabilize the plant
R = 5;
Q = 1;
[K,P] = lqr(A,B,Q,R);

% Nominal LQR
filterCase = 1;

% Default value for safety filter design parameter
eta = 2; 

% Default value for stability
gamma = 1;

% Norm of the uncertainty
beta = 0.1;