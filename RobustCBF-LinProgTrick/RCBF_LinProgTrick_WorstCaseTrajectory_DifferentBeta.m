%% Model
mdl = 'SafetyCriticalLQRWithWorstCaseUncertainty';
load_system(mdl);

%% Robust CBF Safety Filter
filterCase = 4;
betaAll = [0.1 0.4 0.6 0.9]; % Uncertainty level
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
    legendArray{i} = ['Uncertain Plant - Robust CBF \beta = ' num2str(betaAll(i))];
end

%% Save data
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% RCBF trajectories
plt = cell(Nb,1);
for i = 1:Nb
    xplt = xRCBF{i};
    plt{i} = plot(xplt(:,1),xplt(:,2),'LineWidth',2);
end

% Unsafe location
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p2 = patch(Xc,Yc,'r','LineWidth',2);
p2.EdgeColor = 'r';

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([plt{:}],legendArray{:},'FontSize',12,'Location','northwest');

%% Plot inputs
figure;hold on;box on;grid on;
for i = 1:Nb
    uplt = RCBFLOGS{i}.u;
    utime = RCBFLOGS{i}.Time;
    plt{i} = plot(utime,uplt,'LineWidth',2);
end
legend([plt{:}],legendArray{:},'Location','southeast');
xlabel('Time (sec)');ylabel('Commanded control input u');

%% Plot inputs
figure;hold on;box on;grid on;
for i = 1:Nb
    hplt = RCBFLOGS{i}.h;
    htime = RCBFLOGS{i}.Time;
    plt{i} = plot(htime,hplt,'LineWidth',2);
end
legend([plt{:}],legendArray{:},'Location','southeast');
xlabel('Time (sec)');ylabel('h(t)');