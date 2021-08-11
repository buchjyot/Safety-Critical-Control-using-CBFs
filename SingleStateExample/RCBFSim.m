%% Model
mdl = 'SafetyCriticalLQR';
load_system(mdl);

%% LQR Simulation
filterCase = 1; %#ok<*NASGU>
simout = sim(mdl);
LQRLOGS = logsout2struct(simout.logsout);

%% CBF Safety Filter
filterCase = 2;
simout = sim(mdl);
CBFLOGS = logsout2struct(simout.logsout);

%% Robust CBF Safety Filter
filterCase = 4;
beta = 0.9;
simout = sim(mdl);
RCBFLOGS = logsout2struct(simout.logsout);
    
%% Save data
save(mfilename);

%% Plot trajectories in state-space
figure;hold on;grid on;box on;

% Closedloop trajectories
p1 = plot(LQRLOGS.Time,LQRLOGS.x,'b','LineWidth',2);

% RCBF trajectories
p2 = plot(CBFLOGS.Time,CBFLOGS.x,'r-.','LineWidth',2);

% Closedloop trajectories
p3 = plot(RCBFLOGS.Time,RCBFLOGS.x,'--g','LineWidth',2);

% Update plot
ylabel('x','FontSize',14);
xlabel('t (sec)','FontSize',14);
legend([p1 p2 p3],'LQR Baseline','Nominal CBF','Robust CBF','FontSize',12,'Location','northwest');

%% Plot inputs
figure;hold on;box on;grid on;
p1 = plot(CBFLOGS.Time,CBFLOGS.u,'r-.','LineWidth',2);
p2 = plot(RCBFLOGS.Time,RCBFLOGS.u,'--g','LineWidth',2);
legend([p1,p2],'Nominal CBF','Robust CBF','Location','southeast');
xlabel('t');ylabel('u(t)');

%% Plot inputs
figure;hold on;box on;grid on;
p1 = plot(CBFLOGS.Time,CBFLOGS.wcw,'r-.','LineWidth',2);
p2 = plot(RCBFLOGS.Time,RCBFLOGS.wcw,'--g','LineWidth',2);
legend([p1,p2],'Nominal CBF','Robust CBF','Location','southeast');
xlabel('t');ylabel('wcw(t)');