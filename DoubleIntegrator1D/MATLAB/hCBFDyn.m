function xdot = hCBFDyn(t,x,params) %#ok<INUSL>
%% hCBFDyn Function
% This function solves the CBF QP at each point integration step.

% Read params
A = params.A;
B = params.B;
K = params.K;
cT = params.cT;
rT = params.rT;
alpha = params.alpha; % Increase the factor to reduce the conservatism

% Nominal control (LQR)
u0 = -K*x;

% Barrier function
h = (x(1)-cT(1))^2 + (x(2)-cT(2))^2 - rT^2;

% Inequality Constraints for QP
% Aineq'*u <= bineq
Aineq = -2*(x(2)-cT(2));
bineq = 2*(x(1)-cT(1))*x(2) + alpha*h;

% Solve QP
SolveQPCase = 2;
switch SolveQPCase
    case 1
        % Numerical Solution
        H = 1;
        f = -u0;
        opt = optimoptions('quadprog','Display','off');
        u = quadprog(H,f,Aineq,bineq,[],[],[],[],[],opt);
        
    case 2
        % Exact Solution
        rho = -bineq+Aineq*u0;
        if rho<=0
            zopt = 0;
        else
            zopt = -rho*Aineq/Aineq^2;
        end
        u = zopt+u0;
end

% Output xdot
xdot = A*x + B*u;
end