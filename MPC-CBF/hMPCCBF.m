%% hMPCCBF
function [feas, xopt, uopt, Jopt] = hMPCCBF(G,x0,Q,R,F,Np,Nc,rk,ubnd,xbnd,alpha,rT,cT)
% Solves a linear Model Predictive Control problem with barrier constarints
% and returns the optimal states and input.
%
% Inputs:
% G is a discrete time LTI system specified by ss object
% r is a reference input
% Np is a prediction horizon
% Nc is a control horizon
% ubnd is a bound on control
% xbnd is a bound on states
% alpha is a design parameter for CBF
% rT and cT are radius and center of the obstacle
%
% Outputs:
% xk is a optimal states
% uk is a optimal input
%
% Assumes that C is identity, i.e. all states are available for feedback.

% State-space matrices
[A,B,C,D,Ts] = ssdata(G);

% Throw error if model is not discrete-time
if isequal(Ts, 0)
    error('Input must be discrete-time LTI model.');
end

% Sizes
Nx = order(G);
Nu = size(B,2);

% Sort bounds
xbnd = sort(xbnd);
ubnd = sort(ubnd);

% Optimization Problem
prob = optimproblem('ObjectiveSense','minimize');

% Create optimization variables
x = optimvar('x',Nx,Np+1,'LowerBound',xbnd(1),'UpperBound',xbnd(2));
u = optimvar('u',Nu,Np,'LowerBound',ubnd(1),'UpperBound',ubnd(2));

% Current state
prob.Constraints.IC = x(:,1) == x0(:);

% Objective & other constraints
cost = 0;
for k = 1:Np
    % Equality constraints
    prob.Constraints.(['EQConst' num2str(k)]) = x(:,k+1) == A*x(:,k) + B*u(:,k);
    
    % Cost
    yk = C*x(:,k) + D*u(:,k);
    ek = rk - yk; % Tracking error
    uk = u(:,k);  % Control effort
    cost = cost + ek'*Q*ek + uk'*R*uk; % Stage cost
    
    % Inequality constraints - Control barrier function
    h = (x(1,k) - cT(1))^2 + (x(2,k) - cT(2))^2 - rT^2;
    hdot = 2*(x(1,k) - cT(1))*x(2,k) + 2*(x(2,k) - cT(2))*u(1,k);
    prob.Constraints.(['INEQConst' num2str(k)]) = hdot >= -alpha*h;
    %hnext = (x(1,k+1) - cT(1))^2 + (x(2,k+1) - cT(2))^2 - rT^2;
    %prob.Constraints.(['INEQConst' num2str(k)]) = hnext-h >= -alpha*h;
end

% Add terminal cost
yNp1 = C*x(:,Np+1);
eNp1 = rk - yNp1;
cost = cost + eNp1'*F*eNp1;

% Objective
prob.Objective = cost;

% See what problem you are solving
% >> show(prob)

% Solve optimization problem
options = optimoptions(prob,'Display','off');
icpoint = struct('x',repmat(x0,1,Np+1),'u',randn(Nu,Np));
[sol,Jopt,exitflag,output,lambda] = solve(prob,icpoint,'Options',options); %#ok<ASGLU>
feas = double(exitflag);

% Return output
if ~isequal(feas,1)
    disp(exitflag);
    xopt = [];
    uopt = [];
else
    xopt = sol.x(:,1:Nc);
    uopt = sol.u(:,1:Nc);
end
end