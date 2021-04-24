%% Consider a linear model
% States are position and velocity

% Dimentions
Nx = 2;
Nu = 1;

% State-space matrices
A = [...
    0 1;...
    0 0];
B = [0;1];
C = eye(Nx);
D = 0;
P = ss(A,B,C,D);

% Discrete-time model for MPC
Ts = 0.1;
G = c2d(P,Ts);

% Initial conditions
x0 = [10;10];

% Obstacle location
cT = [10,-5];
rT = 4;

% Circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);

%% Solve MPC-CBF Problem
% Prediction Horizon
Np = 10;

% Control Horizon
Nc = 1;

% MPC Cost matrices
Q = eye(Nx);
F = eye(Nx);
R = 1;

% Bounds on states and controls
xbnd = [-Inf, Inf];
ubnd = [-Inf, Inf];

% Reference
TargetStates = [0;0];

% Time
T0 = 0;
Tf = 40;
t = T0:Ts:Tf;
Nt = length(t);

% Memory allocation
xt = zeros(Nx,Nt);
ut = zeros(Nu,Nt);

% Initialization
u0 = zeros(Nu,1);
xt(:,1) = x0;
ut(:,1) = u0;

% Design parameter
alpha = 2;

% Solve MPC problem
warning('off','Control:analysis:LsimStartTime');
for k = 1:Nt-1
    
    % Determine optimal control input
    [feas, xopt, uopt, Jopt] = hMPCCBF(G,xt(:,k),Q,R,F,Np,Nc,TargetStates,ubnd,xbnd,alpha,rT,cT);
    
    % Simulate plant by holding control input over small interval
    [Y,T,X] = lsim(P,uopt*ones(1,2),[t(k) t(k+1)],xt(:,k));
    
    % Update    
    xt(:,k+1) = X(end,:);
    ut(:,k+1) = uopt;

    % Print
    fprintf('t = %.2f, x1 = %.4f, x2 = %.4f, uopt = %.4f \n',t(k),X(end,1),X(end,2),uopt);
    
    % Stop integration when near to origin
    if norm(xt(:,k+1)) <= 5e-3
        break;
    end
end
warning('on','Control:analysis:LsimStartTime')

%% Save data
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Initial condition
plot(x0(1),x0(2),'bo','MarkerFaceColor','b','LineWidth',2);

% Unsafe location
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p3 = patch(Xc,Yc,'r','LineWidth',2);
p3.EdgeColor = 'r';

% CLF-CBF trajectories
p5 = plot(xt(1,:),xt(2,:),'g','LineWidth',2);

% Update plot
clear alpha;
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p3 p5],'Unsafe Region','MPC-CBF-QP','FontSize',12,'Location','northwest');