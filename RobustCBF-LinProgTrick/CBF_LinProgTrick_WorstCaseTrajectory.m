%% Model
mdl1 = 'SafetyCriticalLQR';
load_system(mdl1);

mdl2 = 'SafetyCriticalLQRWithWorstCaseUncertainty';
load_system(mdl2);

%% Robust CBF Safety Filter
filterCase = 4;

% Plant with no uncertainty
beta = 0.9;
simout = sim(mdl1);
RCBFLOGS1 = logsout2struct(simout.logsout);

% Plant with worst-case uncertainty
simout = sim(mdl2);
RCBFLOGS2 = logsout2struct(simout.logsout);

%% Save data
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% RCBF trajectories
p1 = plot(RCBFLOGS1.x(:,1),RCBFLOGS1.x(:,2),'b','LineWidth',2);
plot(RCBFLOGS1.x(1,1),RCBFLOGS1.x(1,2),'bo','MarkerFaceColor','b','LineWidth',2);

% Worst-case trajectory
p2 = plot(RCBFLOGS2.x(:,1),RCBFLOGS2.x(:,2),'c','LineWidth',2);

% Unsafe location
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p3 = patch(Xc,Yc,'r','LineWidth',2);
p3.EdgeColor = 'r';

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
title('RobustCBF Design');
legend([p1 p2 p3],'No uncertainty','Worst-case uncertainty','Unsafe Region','FontSize',12,'Location','northwest');