%% Model
mdl1 = 'SafetyCriticalLQR';
load_system(mdl1);

mdl2 = 'SafetyCriticalLQRWithWorstCaseUncertainty';
load_system(mdl2);

% Uncertainty level
beta = 0.6;

%% Nominal CBF Safety Filter
filterCase = 2; %#ok<*NASGU>

% Nominal plant, Nominal CBF controller
simout = sim(mdl1);
CBFLOGS1 = logsout2struct(simout.logsout);
xCBF1 = CBFLOGS1.x;

% Uncertain plant, Nominal CBF controller
simout = sim(mdl2);
CBFLOGS2 = logsout2struct(simout.logsout);
xCBF2 = CBFLOGS2.x;

%% Robust CBF Safety Filter
filterCase = 4;

% Nominal Plant, Robust CBF controller
simout = sim(mdl1);
RCBFLOGS1 = logsout2struct(simout.logsout);
xRCBF1 = RCBFLOGS1.x;

% Uncertain Plant, Robust CBF controller
simout = sim(mdl2);
RCBFLOGS2 = logsout2struct(simout.logsout);
xRCBF2 = RCBFLOGS2.x;

%% Save data
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Closedloop trajectories
p3 = plot(xCBF1(:,1),xCBF1(:,2),'LineWidth',2);
p4 = plot(xCBF2(:,1),xCBF2(:,2),'LineWidth',2);
p5 = plot(xRCBF1(:,1),xRCBF1(:,2),'LineWidth',2);
p6 = plot(xRCBF2(:,1),xRCBF2(:,2),'LineWidth',2);

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
legend([p2,p3,p4,p5,p6],'Unsafe Region','Nominal Plant - Nominal CBF',...
    'Uncertain Plant - Nominal CBF',...
    'Nominal Plant - Robust CBF',....
    'Uncertain Plant - Robust CBF',...
    'FontSize',12,'Location','northwest');

%% Plot h
figure;hold on;grid on;box on;
p3 = plot(CBFLOGS1.Time,CBFLOGS1.h,'LineWidth',2);
p4 = plot(CBFLOGS2.Time,CBFLOGS2.h,'LineWidth',2);
p5 = plot(RCBFLOGS1.Time,RCBFLOGS1.h,'LineWidth',2);
p6 = plot(RCBFLOGS2.Time,RCBFLOGS2.h,'LineWidth',2);
ylabel('h(t)','FontSize',12);
xlabel('t (sec)','FontSize',12);
legend('Nominal Plant - Nominal CBF',...
    'Uncertain Plant - Nominal CBF',...
    'Nominal Plant - Robust CBF',....
    'Uncertain Plant - Robust CBF','FontSize',12,'Location','northeast');