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
odeParams = struct('A',A,'B',B,'cT',cT,'rT',rT,'P',P,'K',K,'x0',x0,'beta',0.8,'plant_case',1);

% ode options
odeOpt = odeset('Events',@originCapture,'RelTol',1e-6,'AbsTol',1e-6);
Tspan = [0 50];

%% Design CLF-ECBF Controller using QP
[tCLFECBF,xCLFECBF,TE1,YE1,IE1] = ode45(@(t,x) hRobustECBFSOCP(t,x,odeParams),Tspan,x0,odeOpt);
save(mfilename);

%% Plot trajectories in state-space
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
legend([p2 p3 p4],'LQR Baseline','Unsafe Region','RobustECBF-UncertainPlant','FontSize',12,'Location','northwest');

%% hRobustECBFSOCP
function xdot = hRobustECBFSOCP(t,x,params) %#ok<INUSL>
% Read params
K = params.K;
cT = params.cT;
rT = params.rT;
beta = params.beta;
A = params.A;
B = params.B;
plant_case = params.plant_case;

% Nominal control
u0 = -K*x;

% Barrier function h and hdot
h = (x(1)-cT(1))^2 + (x(2)-cT(2))^2 - rT^2;
hdot = 2*(x(1)-cT(1))*x(3) + 2*(x(2)-cT(2))*x(4);

% Choose poles
p1 = 1;
p2 = 2;
Kalpha = [p1*p2 p1+p2];

% Define LgLfh
LgLfh1 = 2*(x(1)-cT(1));
LgLfh2 = 2*(x(2)-cT(2));
LgLfh = [LgLfh1 LgLfh2];

% Cost term
f = [-u0' 1];

% Dimentions
Nu = size(B,2);
ZNu = zeros(Nu,1);

% SOC1
A1 = [blkdiag(0,1) ZNu; ZNu' 0];
b1 = [ZNu; 0];
d1 = [LgLfh'/(beta*norm(LgLfh2)); 0];
gamma1 = -1/(beta*norm(LgLfh2))*(2*x(3)^2 + 2*x(4)^2 + Kalpha*[h;hdot]);
socConstraints(1) = secondordercone(A1,b1,d1,gamma1); %#ok<*EMVDF>

% SOC2
A2 = [eye(Nu) ZNu; ZNu' 1/sqrt(2)];
b2 = [ZNu; 1/sqrt(2)];
d2 = [ZNu; 1/sqrt(2)];
gamma2 = -1/sqrt(2);
socConstraints(2) = secondordercone(A2,b2,d2,gamma2);

% Solve SOC
options = optimoptions('coneprog','MaxIterations',100,'Display','off');
sol = coneprog(f,socConstraints,[],[],[],[],[],[],options);
u = sol(1:Nu);

% Worst-case w
wwc = -beta*LgLfh2'*norm(u(2))/norm(LgLfh2);
disp([u(2) wwc]);

% Compute xdot
switch plant_case
    case 0
        % (Nominal Model)
        xdot = A*x + B*u;
        
    case 1
        % (Worst-Case Uncertain Model)
        xdot = A*x + B(:,1)*u(1) + B(:,2)*(u(2) + wwc);
end
end

%% originCapture Function
function [value,isterminal,direction] = originCapture(t,x) %#ok<INUSL>
% This function defines capture event to stop the simulation

% We want to bring the states to origin
StopRadius    = 1e-3;
value(1)      = norm(x(1:2)) - StopRadius;
isterminal(1) = 1;   % we want to end integration when collision occurs
direction(1)  = 0;   % The zero should only ever be approached from above
end