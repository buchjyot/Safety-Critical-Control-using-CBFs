%% Consider a linear model
% States are 2D position, 2D velocity
% i.e. x = [xPos,yPos,xVel,yVel]

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
D = 0;
G = ss(A,B,C,D);

% Initial conditions
x0 = [-5;-5;0;0];

% Obstacle location
cT = [-2,-2.5];
rT = 1.5;

%% Infinite Horizon LQR Design
Q = blkdiag(1,1,10,10);
R = eye(2);
[K,P] = lqr(A,B,Q,R);
CL = feedback(G,K);
[yCL,tCL,xCL] = initial(CL,x0);

% params for ODE integration
odeParams = struct('A',A,'B',B,'cT',cT,'rT',rT,'P',P,'x0',x0);

% ode options
odeOpt = odeset('Events',@originCapture,'RelTol',1e-6,'AbsTol',1e-6);
Tspan = [0 50];

%% Design CLF-ECBF Controller using QP
[tCLFECBF,xCLFECBF,TE1,YE1,IE1] = ode45(@(t,x) hCLFECBFDyn(t,x,odeParams),Tspan,x0,odeOpt);
save(mfilename);
return;

%% Plot trajectories in state-space
load('CLFECBFExample.mat');
figure;hold on;grid on;box on;

% Closedloop trajectories
p2 = plot(yCL(:,1),yCL(:,2),'b','LineWidth',2);

% Circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);

% Unsafe location
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p3 = patch(Xc,Yc,'r','LineWidth',2);
p3.EdgeColor = 'r';

% CBF trajectories
p4 = plot(xCLFECBF(:,1),xCLFECBF(:,2),'g','LineWidth',2);

% Markers
plot(yCL(1,1),yCL(1,2),'ko','MarkerFaceColor','w','LineWidth',2);
plot(0,0,'kx','LineWidth',2,'MarkerSize',8);

% Update plot
xlabel('x [ft]','FontSize',14);
ylabel('y [ft]','FontSize',14);
axis equal;alpha(0.1);
xlim([-6 1]);
ylim([-6 1]);
legend([p2 p3 p4],'LQR Baseline','Unsafe Region','CLF-ECBF','FontSize',12,'Location','northwest');

%% hCLFECBFDyn Function
function xdot = hCLFECBFDyn(t,x,params) %#ok<INUSL>
% This function solves the CLF-ECBF QP at each point integration step.

% Read params
A  = params.A;
B  = params.B;
cT = params.cT;
rT = params.rT;
P  = params.P;
x0 = params.x0; %#ok<NASGU>

% Barrier function
h = (x(1)-cT(1))^2 + (x(2)-cT(2))^2 - rT^2;
hdot = 2*(x(1)-cT(1))*x(3) + 2*(x(2)-cT(2))*x(4);

% Penalty on slack variable
m = 1;

% Design parameter for CLF
gamma = 2;

% Design parameters for ECBF
% The first pole should satisfy p1 >= -hdot(x0)/h(x0) -- Since initial
% conditions x(3) and x(4) are zero, this can be any positive real number.
% The second pole should satisfy p2 >= -(hddot(x0) + p1*hdot(x0))/(hdot(x0) + p1*h(x0))
% This constraint is part of the optimization so you can pick any positive
% real number. Optimization will figure out how to satisfy it.
PerformPolePlacement = true;
if PerformPolePlacement
    p1 = 1;
    p2 = 2;
    Kalpha = place([0 1;0 0],[0; 1],[-p1 -p2]);
else
    % Brute-force choosing of gains
    alpha1 = 2; %#ok<UNRCH>
    alpha2 = 3;
    Kalpha = [alpha1 alpha2];
end

% Storage
V = x'*P*x;

% Equality constraints hddot = mu
% Aeq*[u;mu;delta] = beq
Aeq = [2*(x(1)-cT(1)) 2*(x(2)-cT(2)) -1 0];
beq = -2*x(3)^2 - 2*x(4)^2;

% Inequality Constraints Vdot <= -beta*V
Aineq1 = [2*x'*P*B 0 -1];
bineq1 = -x'*(A'*P + P*A)*x - gamma*V;

% Inequality constraints for mu >= -Kalpha*[h;hdot]
Aineq2 = [zeros(1,2), -1, 0];
bineq2 = Kalpha*[h;hdot];

% Inequality Constraints
% Aineq*[u;mu;delta] <= bineq
Aineq = [Aineq1;Aineq2];
bineq = [bineq1;bineq2];

% Solve QP
H = blkdiag(eye(2),0,m);
f = zeros(4,1);
opt = optimoptions('quadprog','Display','off');
sol = quadprog(H,f,Aineq,bineq,Aeq,beq,[],[],[],opt);

% Output xdot
u = sol(1:2);
% mu = sol(3);
% delta = sol(4);

% Dynamics
xdot = A*x + B*u;
end

%% originCapture Function
function [value,isterminal,direction] = originCapture(t,x) %#ok<INUSL>
% This function defines capture event to stop the simulation

% We want to bring the states to origin
StopRadius    = 5e-3;
value(1)      = norm(x(1:2)) - StopRadius;
isterminal(1) = 1;   % we want to end integration when collision occurs
direction(1)  = 0;   % The zero should only ever be approached from above
end