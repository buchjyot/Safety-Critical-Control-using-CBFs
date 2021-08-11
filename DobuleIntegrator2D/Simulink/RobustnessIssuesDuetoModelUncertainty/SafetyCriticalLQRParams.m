%% Plant
% Double integrator point-mass plant model
% The states are position and velocity.

% Dimentions
Nx = 4;

% State-space matrices
A = [...
    0 0 1 0;...
    0 0 0 1;...
    0 0 0 0;...
    0 0 0 0;
    ];
B = [0 0;0 0;1 0;0 1];
C = eye(4);
D = zeros(4,2);
G = ss(A,B,C,D);

% Initial conditions
x0 = [-5;-5;0;0];

% Drive the states to origin
Ref = [0;0;0;0];

% Simulation time
tf_upperbound = 50;

% Obstacle location
cT = [-2,-2.5];
rT = 1.5;

% Design LQR to stabilize the plant
Q = blkdiag(1,1,10,10);
R = eye(2);
[K,P] = lqr(A,B,Q,R);

% Nominal LQR
filterCase = 1;

% Nominal lag
tau = 0;

% Gamma
gamma = 1;

% Nominal Uncertainty
Delta = ss(0,zeros(1,2),zeros(2,1),zeros(2));