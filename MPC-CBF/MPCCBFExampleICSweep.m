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

% Obstacle location
cT = [10,-5];
rT = 5;

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

% Initial conditions
ic_case = 3;
switch ic_case    
    case 1
        % Try all initial conditions
        x0All = {...
            [0,5],[0,10],[0,-5],[0,-10],...
            [5,5],[5, 10],[5,-15],[5,-10],...
            [10,5],[10,10],[10,-15],...
            [15,5],[15,10],[15,-10],[15,-15]...
            [20,10],[20,5],[20,0],[20,-5],[20,-10],[20,-15],...
            [25,-15],[25,10],[25,5],[25,0],[25,-5],[25,-10],...
            [30,10],[30,5],[30,0],[30,-5],[30,-10],[30,-15],...
            [35,10],[35,5],[35,0],[35,-5],[35,-10],[35,-15],...
            [40,10],[40,5],[40,0],[40,-5],[40,-10],[40,-15],...
            [45,10],[45,5],[45,0],[45,-5],[45,-10],[45,-15],...
            [50,10],[50,5],[50,0],[50,-5],[50,-10]};
        
    case 2
        % Getting stuck case
        x0All = {[10,10]};
        
    case 3
        % Trajectory reaches to the origin
        x0All = {[40,10]};
end

% Memory allocation
Nx0 = length(x0All);
xAll = cell(Nx0,1);
uAll = cell(Nx0,1);

for j = 1:Nx0
    % Initial conditions
    x0 = x0All{j};
    
    try
        % Time
        T0 = 0;
        Tf = 50;
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
        alpha = 1;
        
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
            fprintf('j = %d, t = %.2f, x1 = %.4f, x2 = %.4f, uopt = %.4f \n',j,t(k),X(end,1),X(end,2),uopt);
            
            % Stop integration when near to origin
            if norm(xt(:,k+1)) <= 5e-3
                break;
            end
        end
        warning('on','Control:analysis:LsimStartTime');
        
    catch ME
        fprintf('Error when j = %d\n',j);
    end
    xAll{j} = xt;
    uAll{j} = ut;
end
save(mfilename);
return;

%% Plot trajectories in state-space
load('MPCCBFExampleICSweep.mat');
figure;hold on;grid on;box on;

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
clear alpha;alpha(0.1);

% MPC-CBF trajectories
for i = 1:Nx0
    xt = xAll{i}';
    x0 = x0All{i};
    [nr,nc] = find(xt);
    xt = xt(1:nr(end),:);
    if norm(xt(end,:)) <= 5e-3
        plot(xt(:,1),xt(:,2),'Color',[0 0.39 0],'LineWidth',2);
    else
        plot(xt(:,1),xt(:,2),'g','LineWidth',2);
    end
    plot(x0(1),x0(2),'bo','MarkerFaceColor','b','LineWidth',2);
end
plot(0,0,'ko','MarkerFaceColor','k','LineWidth',2);

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;
xlim([-15 55]);
ylim([-25 20]);
legend(p3,'Unsafe Region','FontSize',12,'Location','northwest');