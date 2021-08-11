%% Model
mdl = 'SafetyCriticalLQRRobustCBF';
load_system(mdl);

%% Simulate
tic;
simout = sim(mdl);
tSimTime = toc;
LOGS = logsout2struct(simout.logsout);

%% Save
save(mfilename);

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = cT(1);
c2 = cT(2);
Xc = c1 + rT*cos(theta);
Yc = c2 + rT*sin(theta);

% Unsafe location
plot(cT(1),cT(2),'ro','MarkerFaceColor','r','LineWidth',2);
p3 = patch(Xc,Yc,'r','LineWidth',2);
p3.EdgeColor = 'r';

% CBF trajectories
xRCBF = LOGS.x;
p4 = plot(xRCBF(:,1),xRCBF(:,2),'g','LineWidth',2);
plot(xRCBF(1,1),xRCBF(1,2),'go','MarkerFaceColor','g');

% Update plot
xlabel('x','FontSize',14);
ylabel('y','FontSize',14);
axis equal;alpha(0.1);
xlim([-50 10]);ylim([-15 15]);
legend([p3 p4],'Unsafe Region','RobustECBF','FontSize',12,'Location','northwest');