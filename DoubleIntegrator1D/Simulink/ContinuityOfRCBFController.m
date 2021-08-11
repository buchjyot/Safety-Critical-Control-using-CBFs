%% Design Parameters
% Dimentions
Nx = 2;

% State-space matrices
A = [0 1;0 0];
B = [0; 1];
C = eye(Nx);
D = [0;0];
G = ss(A,B,C,D);

% Design LQR to stabilize the plant
R = 5;
Q = 0.5*eye(Nx);
[K,P] = lqr(A,B,Q,R);

% Obstacle
cT = [10;-5];
rT = 4;
eta = 2;
beta = 0.9;

% Grid on x2
x2Grid = -10:0.1:10;
NGrid = length(x2Grid);
u = zeros(NGrid,1);
cost = zeros(NGrid,1);
x1 = 15;
for i = 1:NGrid
    x2 = x2Grid(i);
    h = (x1-cT(1))^2 + (x2-cT(2))^2 - rT^2;
    u0 = -K*[x1;x2];
    [u(i),cost(i)] = RobustCBFQPLinProgTick(eta,beta,h,cT,u0,[x1;x2]);
end

%% Plot
figure;grid on;box on;hold on;
plot(x2Grid,u,'b','LineWidth',2);
xlabel('Velocity x_2(t)');
ylabel('Control Input u(t)');
title('Fixed value of Position x_1 = 15');

%% Function
function [u,cost] = RobustCBFQPLinProgTick(eta,beta,h,cT,u0,x)
% Inequality Constraints for QP
% Aineq'*u <= bineq
p = x(2)-cT(2);

% Solve QP
if 2*(x(1)-cT(1))*x(2) + 2*p*(u0 - beta*sign(p)*abs(u0)) >= -eta*h
    % Choose nominal control
    u = u0;
    cost = 0.5*(u0'*u0);
else
    Aineq = [-2*p*(1 - sign(p)*beta) +2*p*(sign(p)*beta + 1); -1 0;0 -1];
    bineq = [2*(x(1)-cT(1))*x(2) + eta*h; 0; 0];
    H = [1 -1;-1 1];
    f = [-u0 +u0];
    opt = optimoptions('quadprog','Algorithm','active-set','Display','off');
    [uStar,cost] = quadprog(H,f,Aineq,bineq,[],[],[],[],[u0;u0]*0,opt);
    cost = cost + 0.5*(u0'*u0);
    
    % up and un
    up = uStar(1);
    un = uStar(2);
    
    % Output
    u = up - un;
end
end