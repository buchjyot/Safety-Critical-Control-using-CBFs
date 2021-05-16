function xdot = hCLFCBFDyn(t,x,params) %#ok<INUSL>
%% hCLFCBFDyn Function
% This function solves CLF-CBF QP at each point integration step.

% Read params
A = params.A;
B = params.B;
P = params.P;
cT = params.cT;
rT = params.rT;
alpha = params.alpha; % Increase the factor to reduce the conservatism
beta = params.beta; % Factor for sacrificing stability

% Barrier function
h = (x(1)-cT(1))^2 + (x(2)-cT(2))^2 - rT^2;

% Inequality Constraints for QP
% Safety: hdot >= -alpha*h
Aineq1 = [-2*(x(2)-cT(2)) 0];
bineq1 = 2*(x(1)-cT(1))*x(2) + alpha*h;

% Stability: Vdot <= -beta*V
Aineq2 = [2*x'*P*B -1];
bineq2 = -x'*(A'*P + P*A)*x - beta*(x'*P*x);

% Aineq'*[u;delta] <= bineq
Aineq = [Aineq1; Aineq2];
bineq = [bineq1; bineq2];

% Solve QP
m = 1;
H = diag([1, m]);
f = [0 0];
opt = optimoptions('quadprog','Display','off');
sol = quadprog(H,f,Aineq,bineq,[],[],[],[],[],opt);

% Read outputs
u = sol(1);
delta = sol(2); %#ok<NASGU>

% Output xdot
xdot = A*x + B*u;
end