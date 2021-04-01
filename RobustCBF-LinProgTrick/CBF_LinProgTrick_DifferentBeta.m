%% Model
mdl = 'SafetyCriticalLQR';
load_system(mdl);

%% LQR Simulation
filterCase = 1; %#ok<*NASGU>
simout = sim(mdl);
LQRLOGS = logsout2struct(simout.logsout);
xLQR = LQRLOGS.x;

%% CBF Safety Filter
filterCase = 2;
simout = sim(mdl);
CBFLOGS = logsout2struct(simout.logsout);
xCBF = CBFLOGS.x;

%% Robust CBF Safety Filter
filterCase = 4;
betaAll = [0 0.1 0.5 0.9]; % Uncertainty level
Nb = length(betaAll);
RCBFLOGS = cell(Nb,1);
xRCBF = cell(Nb,1);
tCBF = cell(Nb,1);
legendArray = cell(Nb,1);
for i = 1:Nb
    beta = betaAll(i);
    simout = sim(mdl);
    RCBFLOGS{i} = logsout2struct(simout.logsout);
    xRCBF{i} = RCBFLOGS{i}.x;
    tCBF{i} = RCBFLOGS{i}.Time;
    legendArray{i} = ['RobustCBF \beta = ' num2str(betaAll(i))];
end

%% Save data
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Closedloop trajectories
p1 = plot(xLQR(:,1),xLQR(:,2),'b','LineWidth',2);
plot(xLQR(1,1),xLQR(1,2),'bo','MarkerFaceColor','b','LineWidth',2);

% Unsafe location
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p2 = patch(Xc,Yc,'r','LineWidth',2);
p2.EdgeColor = 'r';

% RCBF trajectories
plt = cell(Nb,1);
for i = 1:Nb
    xplt = xRCBF{i};
    plt{i} = plot(xplt(:,1),xplt(:,2),'LineWidth',2);
end

% Closedloop trajectories
p3 = plot(xCBF(:,1),xCBF(:,2),'--g','LineWidth',2);

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p1 p2 p3 plt{:}],'LQR Baseline','Unsafe Region','CBF',legendArray{:},'FontSize',12,'Location','northwest');

%% Plot inputs
figure;hold on;box on;grid on;
plot(CBFLOGS.Time,CBFLOGS.u,'b',RCBFLOGS{1}.Time,RCBFLOGS{1}.u,'--r','LineWidth',2);
legend('Nominal CBF','Robust CBF with \beta = 0','Location','southeast');
xlabel('u');ylabel('t');