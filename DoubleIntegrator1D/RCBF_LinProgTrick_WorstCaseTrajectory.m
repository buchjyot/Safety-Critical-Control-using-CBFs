%% Model
mdl1 = 'SafetyCriticalLQR';
load_system(mdl1);

mdl2 = 'SafetyCriticalLQRWithWorstCaseUncertainty';
load_system(mdl2);

mdl3 = 'SafetyCriticalLQRWithWorstCaseUncertaintyNegSign';
load_system(mdl3);

%% Robust CBF Safety Filter
filterCase = 4;
beta = 0.8;

% Plant with no uncertainty
simout = sim(mdl1);
RCBFLOGS1 = logsout2struct(simout.logsout);

% Plant with worst-case uncertainty (+sign)
simout = sim(mdl2);
RCBFLOGS2 = logsout2struct(simout.logsout);

% Plant with worst-case uncertainty (-sign)
simout = sim(mdl3);
RCBFLOGS3 = logsout2struct(simout.logsout);

%% Save data
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% RCBF trajectories
p1 = plot(RCBFLOGS1.x(:,1),RCBFLOGS1.x(:,2),'b','LineWidth',2);
plot(RCBFLOGS1.x(1,1),RCBFLOGS1.x(1,2),'bo','MarkerFaceColor','b','LineWidth',2);

% Worst-case trajectory
p2 = plot(RCBFLOGS2.x(:,1),RCBFLOGS2.x(:,2),'m','LineWidth',2);

% Worst-case trajectory
p3 = plot(RCBFLOGS3.x(:,1),RCBFLOGS3.x(:,2),'c','LineWidth',2);

% Unsafe location
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p4 = patch(Xc,Yc,'r','LineWidth',2);
p4.EdgeColor = 'r';

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-30 40]);
ylim([-20 25]);
title('Robust CBF Design');
legend([p1 p2 p3 p4],'No uncertainty','Sim with +\Delta_{wc}','Sim with -\Delta_{wc}','Unsafe Region','FontSize',12,'Location','northwest');

%% Plot Control Effort
figure,hold on;box on;grid on;
p1 = plot(RCBFLOGS1.Time,RCBFLOGS1.u,'b',RCBFLOGS2.Time,RCBFLOGS2.u,'m',RCBFLOGS3.Time,RCBFLOGS3.u,'c','LineWidth',2);
legend('No uncertainty','Sim with +\Delta','Sim with -\Delta','Location','southeast');
xlabel('Time (sec)');
ylabel('Control input u');
title('\beta = 0.8');

%% Plot h
figure;hold on;grid on;box on;
plot(RCBFLOGS1.Time,RCBFLOGS1.h,'b','LineWidth',2);
plot(RCBFLOGS2.Time,RCBFLOGS2.h,'m','LineWidth',2);
plot(RCBFLOGS3.Time,RCBFLOGS3.h,'c','LineWidth',2);
legend('No uncertainty','Sim with +\Delta','Sim with -\Delta','Location','southeast');
ylabel('h(t)','FontSize',12);
title('\beta = 0.8');
xlabel('t (sec)','FontSize',12);