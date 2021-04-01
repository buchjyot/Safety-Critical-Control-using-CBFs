%% Model
mdl = 'SafetyCriticalLQR';
load_system(mdl);

%% LQR Simulation
filterCase = 1; %#ok<NASGU>
simout = sim(mdl);
LQRLOGS = logsout2struct(simout.logsout);
xLQR = LQRLOGS.x;

%% CBF Safety Filter
filterCase = 2;
simout = sim(mdl);
CBFLOGS = logsout2struct(simout.logsout);
xCBF = CBFLOGS.x;

%% Monte-Carlo
NMC = 50;
MCLOGS = cell(NMC,1);
DelNorm = 0.1;
DelOrder = 1;
DelSamples = ltiusample(DelNorm,NMC,DelOrder);
for i = 1:NMC
    disp(i);
    Delta = ss(DelSamples{i});
    simout = sim(mdl);
    MCLOGS{i} = logsout2struct(simout.logsout);
end

%% Save
save(mfilename);
return;

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Closedloop trajectories
p2 = plot(xLQR(:,1),xLQR(:,2),'b','LineWidth',2);
plot(xLQR(1,1),xLQR(1,2),'bo','MarkerFaceColor','b','LineWidth',2);

% CBF Monte-Carlo
for i = 1:NMC
    xMC = MCLOGS{i}.x;
    p4 = plot(xMC(:,1),xMC(:,2),'g','LineWidth',2);
end

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
p5 = plot(xCBF(:,1),xCBF(:,2),'m','LineWidth',2);

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p2 p3 p4 p5],'LQR Baseline','Unsafe Region','Monte-Carlo Sims','Nominal CBF-QP','FontSize',12,'Location','northwest');