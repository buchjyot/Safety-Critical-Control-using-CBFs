function u = hMATLABfcn(Input)
% Read inputs
beta = Input(1);
cT = Input(2:3);
rT = Input(4);
u0 = Input(5:6);
x = Input(7:10);

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
Nu = 2;
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