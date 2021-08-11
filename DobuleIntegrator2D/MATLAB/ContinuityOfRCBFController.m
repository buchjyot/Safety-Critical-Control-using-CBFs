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
params = struct('A',A,'B',B,'cT',cT,'rT',rT,'P',P,'K',K,'x0',x0,'beta',0.8);

% Grid on x2
x1Grid = -10:0.2:10;
x2Grid = -10:0.2:10;
Nx1Grid = length(x1Grid);
Nx2Grid = length(x2Grid);
u = zeros(Nx1Grid,Nx2Grid,2);
for i = 1:Nx1Grid
    x1 = x1Grid(i);
    for j = 1:Nx2Grid
        x2 = x2Grid(j);
        u(i,j,:) = hRobustECBFSOCP([x1;x2;0;0],params);
    end
end

%% Save
save(mfilename);

%% Plot
for id = 26   
    figure(1);grid on;box on;hold on;
    plot(x1Grid,squeeze(u(:,id,1)),'-',x1Grid,squeeze(u(:,id,2)),'--','LineWidth',2)
    legend('u_x','u_y','Location','southwest');
    xlabel('Y-Position');
    ylabel('Control Input u(t)'); 
    title(sprintf('Fixed value of Y-Position = %.1f',x2Grid(id)));
    
    figure(2);grid on;box on;hold on;
    plot(x2Grid,squeeze(u(id,:,1)),'-',x2Grid,squeeze(u(id,:,2)),'--','LineWidth',2)
    legend('u_x','u_y','Location','southwest');
    xlabel('X-Position');
    ylabel('Control Input u(t)'); 
    title(sprintf('Fixed value of X-Position = %.1f',x2Grid(id)));
end

%% hRobustECBFSOCP
function u = hRobustECBFSOCP(x,params)
% Read params
K = params.K;
cT = params.cT;
rT = params.rT;
beta = params.beta;
A = params.A;
B = params.B;

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
end