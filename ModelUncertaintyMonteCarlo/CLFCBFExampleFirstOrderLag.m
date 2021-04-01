%% Model
mdl = 'SafetyCriticalLQRFirstOrderLag';
load_system(mdl);
filterCase = 3;

%% CLFCBF Safety Filter tau = 0
tau = 0;
simout = sim(mdl);
CLFCBFLOGS0 = logsout2struct(simout.logsout);
xCLFCBF0 = CLFCBFLOGS0.x;

%% CLFCBF Safety Filter tau = 0.5
tau = 0.5;
simout = sim(mdl);
CLFCBFLOGS1 = logsout2struct(simout.logsout);
xCLFCBF1 = CLFCBFLOGS1.x;

%% Save
save(mfilename);

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Closedloop trajectories
plot(x0(1),x0(2),'bo','MarkerFaceColor','b','LineWidth',2);

% Unsafe location: circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p3 = patch(Xc,Yc,'r','LineWidth',2);
p3.EdgeColor = 'r';

% CLF-CBF trajectories
p5 = plot(xCLFCBF0(:,1),xCLFCBF0(:,2),'m','LineWidth',2);
p6 = plot(xCLFCBF1(:,1),xCLFCBF1(:,2),'g','LineWidth',2);

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p3 p5 p6],'Unsafe Region','CLF-CBF-QP \tau = 0','CLF-CBF-QP \tau = 0.5','FontSize',12,'Location','northwest');