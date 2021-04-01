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

%% CLF-CBF Design
filterCase = 3;
simout = sim(mdl);
CLFCBFLOGS = logsout2struct(simout.logsout);
xCLFCBF = CLFCBFLOGS.x;

%% Save Data
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Closedloop trajectories
p1 = plot(xLQR(:,1),xLQR(:,2),'b','LineWidth',2);
plot(xLQR(1,1),xLQR(1,2),'bo','MarkerFaceColor','b','LineWidth',2);

% Unsafe location: circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p2 = patch(Xc,Yc,'r','LineWidth',2);
p2.EdgeColor = 'r';

% CBF trajectories
p3 = plot(xCBF(:,1),xCBF(:,2),'g','LineWidth',2);

% CLFCBF trajectories
p4 = plot(xCLFCBF(:,1),xCLFCBF(:,2),'m','LineWidth',2);

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p1 p2 p3 p4],'LQR Baseline','Unsafe Region','CBF-QP','CLF-CBF-QP','FontSize',12,'Location','northwest');