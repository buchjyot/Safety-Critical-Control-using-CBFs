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

% Initial conditions
x0 = [10;10];

% Obstacle location
cT = [10;-5];
rT = 5;

% Circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);

%% Open-loop Simulation
[yOL,tOL,xOL] = initial(G,x0);

%% Nominal Control Design
% Design LQR to stabilize the plant
R = 5;
Q = 0.5*eye(Nx);
[K,P] = lqr(A,B,Q,R);
CL = feedback(G,K);
[yCL,tCL,xCL] = initial(CL,x0);

% params for ODE integration
odeParams = struct('A',A,'B',B,'K',K,'cT',cT,'rT',rT,'P',P,'alpha',2,'beta',1);

% ode options
odeOpt = odeset('Events',@hOriginCaptureEvent,'RelTol',1e-6,'AbsTol',1e-6);

%% Design CBF
[tCBF,xCBF,TE1,YE1,IE1] = ode45(@(t,x) hCBFDyn(t,x,odeParams),[0 500],x0,odeOpt);

%% Design CLF-CBF
[tCLFCBF,xCLFCBF,TE2,YE2,IE2] = ode45(@(t,x) hCLFCBFDyn(t,x,odeParams),[0 500],x0,odeOpt);

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Openloop trajectories
p1 = plot(yOL(:,1),yOL(:,2),'k','LineWidth',2);

% Closedloop trajectories
p2 = plot(yCL(:,1),yCL(:,2),'b','LineWidth',2);
plot(yCL(1,1),yCL(1,2),'bo','MarkerFaceColor','b','LineWidth',2);

% Unsafe location
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p3 = patch(Xc,Yc,'r','LineWidth',2);
p3.EdgeColor = 'r';

% CBF trajectories
p4 = plot(xCBF(:,1),xCBF(:,2),'g','LineWidth',2);

% CLF-CBF trajectories
p5 = plot(xCLFCBF(:,1),xCLFCBF(:,2),'.-m','LineWidth',2);

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p1 p2 p3 p4 p5],'Openloop','LQR Baseline','Unsafe Region','CBF-QP','CLF-CBF-QP','FontSize',12,'Location','northwest');