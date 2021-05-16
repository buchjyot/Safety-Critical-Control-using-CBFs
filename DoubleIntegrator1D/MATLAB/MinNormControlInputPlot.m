%% Plant
% Double integrator point-mass plant model
% The states are position and velocity.

% Dimentions
Nx = 2;

% State-space matrices
A = [0 1;0 0];
B = [0; 1];
C = eye(Nx);
D = 0;
G = ss(A,B,C,D);

%% Nominal Control Design
% Design LQR to stabilize the plant
R = 5;
Q = 0.5*eye(Nx);
[K,P] = lqr(A,B,Q,R);

%% Grid on State-Space
params = struct('A',A,'B',B,'P',P,'gamma',2);
x1All = -5:0.1:5;
x2All = -5:0.1:5;
Ni = length(x1All);
Nj = length(x2All);
xAll = zeros(2,Ni,Nj);
uAll = zeros(Ni,Nj);
for i = 1:Ni
    for j = 1:Nj
        xAll(:,i,j) = [x1All(i);x2All(j)];
        uAll(i,j) = minNormController(xAll(:,i,j),params);
    end
end
save(mfilename);

%% Plot
h = surf(x1All,x2All,uAll);
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
zlabel('u','FontSize',14);
zlim([-10 10])

%% Function minNormController
function u = minNormController(x,params)
%% minNormController

% Read params
A = params.A;
B = params.B;
P = params.P;
gamma = params.gamma; % Increase the factor to reduce the conservatism

% Barrier function
V = x'*P*x;

% Inequality Constraints for QP
% Aineq'*u <= bineq
Aineq = 2*x'*P*B;
bineq = -x'*(A'*P + P*A)*x - gamma*V;

% Numerical Solution
H = 1;
opt = optimoptions('quadprog','Display','off');
u = quadprog(H,[],Aineq,bineq,[],[],[],[],[],opt);
end