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
rT = 4;

% Circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);

%% Nominal Control Design
% Design LQR to stabilize the plant
R = 5;
Q = 0.5*eye(Nx);
[K,P] = lqr(A,B,Q,R);
CL = feedback(G,K);
[yCL,tCL,xCL] = initial(CL,x0);

% ode options
odeOpt = odeset('Events',@hOriginCaptureEvent,'RelTol',1e-6,'AbsTol',1e-6);

%% Design CBF

% Uncertainty level
beta = [0.6];

% Plant flag for uncertain or nominal plant
plant_flag = 1;

% Loop over uncertainty-level
Nb = length(beta);
xCBF = cell(2*Nb,1);
tCBF = cell(2*Nb,1);
legendArray = cell(2*Nb,1);
for i = 1:Nb
    odeParams = struct('A',A,'B',B,'K',K,'cT',cT,'rT',rT,'P',P,'alpha',2,'beta',beta(i),'plant_flag',plant_flag,'OP','QP');
    [tCBF{2*i-1},xCBF{2*i-1}] = ode45(@(t,x) LOCALCBFDyn(t,x,odeParams),[0 50],x0,odeOpt);
    legendArray{2*i-1} = ['RobustCBF (QP) \beta = ' num2str(beta(i))];
    
    odeParams = struct('A',A,'B',B,'K',K,'cT',cT,'rT',rT,'P',P,'alpha',2,'beta',beta(i),'plant_flag',plant_flag,'OP','SOCP');
    [tCBF{2*i},xCBF{2*i}] = ode45(@(t,x) LOCALCBFDyn(t,x,odeParams),[0 50],x0,odeOpt);
    legendArray{2*i} = ['RobustCBF (SOCP) \beta = ' num2str(beta(i))];
end

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Closedloop trajectories
p1 = plot(yCL(:,1),yCL(:,2),'b','LineWidth',2);
plot(yCL(1,1),yCL(1,2),'bo','MarkerFaceColor','b','LineWidth',2);

% Unsafe location
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p2 = patch(Xc,Yc,'r','LineWidth',2);
p2.EdgeColor = 'r';

% CBF trajectories
plt = cell(Nb,1);
for i = 1:Nb
    xplt = xCBF{2*i-1};
    plt{2*i-1} = plot(xplt(:,1),xplt(:,2),'LineWidth',2);
  
    xplt = xCBF{2*i};
    plt{2*i} = plot(xplt(:,1),xplt(:,2),'--','LineWidth',2);
end

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p1 p2 plt{:}],'LQR Baseline','Unsafe Region',legendArray{:},'FontSize',12,'Location','northwest');

%% LOCALCBFDyn Function
function xdot = LOCALCBFDyn(t,x,params)
% This function solves the CBF QP at each point integration step.

% Read params
A = params.A;
B = params.B;
K = params.K;
cT = params.cT;
rT = params.rT;
alpha = params.alpha; % Increase the factor to reduce the conservatism
beta = params.beta; % Uncertainty norm bound
plant_flag = params.plant_flag; % 0 for nominal, 1 for uncertain

% Nominal control (LQR)
u0 = -K*x;

% Barrier function
h = (x(1)-cT(1))^2 + (x(2)-cT(2))^2 - rT^2;

% Define p
p = x(2)-cT(2);

% Check if nominal control satisfies barrier constraint
if  2*(x(1)-cT(1))*x(2) + 2*p*(u0 - beta*sign(p)*abs(u0)) >= -alpha*h
    % Choose nominal control
    u = u0;
    
    % Print values
    fprintf('t = %.4f, u = u0 = %.4f\n',t,u0);
else
    switch params.OP
        case 'QP'
            % Inequality Constraints for QP
            % Aineq'*u <= bineq
            Aineq = [-2*p*(1 - sign(p)*beta) +2*p*(sign(p)*beta + 1); -1 0;0 -1];
            bineq = [2*(x(1)-cT(1))*x(2) + alpha*h; 0; 0];

            % Solve QP
            H = [1 -1;-1 1];
            f = [-2*u0 +2*u0];
            opt = optimoptions('quadprog','Display','off');
            uStar = quadprog(H,f,Aineq,bineq,[],[],[],[],[],opt);

            % up and un
            up = uStar(1);
            un = uStar(2);

            % Print values of up and un
            fprintf('t = %.4f, up = %.4f, un = %.4f, u0 =  %.4f\n',t,up,un,u0);

            % Output xdot
            u = up - un;
        case 'SOCP'
            f = [-u0' 1];
            
            Nu = size(B,2);
            Znu1 = zeros(Nu,1);
            
            A1 = [eye(Nu) Znu1; Znu1' 0];
            b1 = [Znu1; 0];
            d1 = [1/(beta*norm(2*p))*2*p'; 0];
            gamma1 = -1/(beta*norm(2*p))*(2*(x(1)-cT(1))*x(2) + alpha*h);
            socConstraints(1) = secondordercone(A1,b1,d1,gamma1);
            
            A2 = [eye(Nu) Znu1; Znu1' 1/sqrt(2)];
            b2 = [Znu1; 1/sqrt(2)];
            d2 = [Znu1; 1/sqrt(2)];
            gamma2 = -1/sqrt(2);
            socConstraints(2) = secondordercone(A2,b2,d2,gamma2);
            
            options = optimoptions('coneprog','MaxIterations',100);
            xopt = coneprog(f,socConstraints,[],[],[],[],[],[],options);
            u = xopt(1:Nu);
    end
end

if plant_flag
    % Worst-case w
    wwc = -beta*abs(u)*sign(p);
else
    % Uncertain input = 0
    wwc = 0;
end

% Dynamics
xdot = A*x + B*(u+wwc);
end

function [value,isterminal,direction] = hOriginCaptureEvent(t,x) %#ok<INUSL>
%% hOriginCaptureEvent Function
% This function defines capture event to stop the simulation

% We want to bring the states to origin
FinalPosition = 0;
FinalVelocity = 0;
value(1)      = (x(1)-FinalPosition)^2 + (x(2)-FinalVelocity)^2 - sqrt(eps);
isterminal(1) = 1;   % we want to end integration when collision occurs
direction(1)  = 0;   % The zero should only ever be approached from above
end