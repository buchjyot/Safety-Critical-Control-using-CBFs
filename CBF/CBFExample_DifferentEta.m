%% Model
mdl = 'SafetyCriticalLQR';
load_system(mdl);

%% LQR Simulation
filterCase = 1;
simout = sim(mdl);
LQRLOGS = logsout2struct(simout.logsout);
xLQR = LQRLOGS.x;

%% CBF Safety Filter
filterCase = 2;

% Change alpha values
etaAll = [0.5,1,2]; %#ok<*NASGU>

% Memory allocation
Na = length(etaAll);
CBFLOGS = cell(Na,1);
xCBF = cell(Na,1);
legendArray = cell(Na,1);
for i = 1:Na
    eta = etaAll(i);
    simout = sim(mdl);
    CBFLOGS{i} = logsout2struct(simout.logsout);
    xCBF{i} = CBFLOGS{i}.x;
    legendArray{i} = ['CBF-QP \eta = ' num2str(eta)];
end

%% Save Data
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Closedloop trajectories
p2 = plot(xLQR(:,1),xLQR(:,2),'b','LineWidth',2);
plot(xLQR(1,1),xLQR(1,2),'bo','MarkerFaceColor','b','LineWidth',2);

% Unsafe location: circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p3 = patch(Xc,Yc,'r','LineWidth',2);
p3.EdgeColor = 'r';

% CBF trajectories
plt = cell(Na,1);
for i = 1:Na
    xCBFi = xCBF{i};
    plt{i} = plot(xCBFi(:,1),xCBFi(:,2),'LineWidth',2);
end

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p2 p3 plt{:}],'LQR Baseline','Unsafe Region',legendArray{:},'FontSize',12,'Location','northwest');