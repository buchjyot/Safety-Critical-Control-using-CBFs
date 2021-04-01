%% Model
mdl = 'SafetyCriticalLQR';
load_system(mdl);

%% LQR Simulation
filterCase = 1; %#ok<*NASGU>
simout = sim(mdl);
LQRLOGS = logsout2struct(simout.logsout);
xLQR = LQRLOGS.x;

%% CLF-CBF Design
filterCase = 3;

% Change alpha values
gammaAll = [0.5,1,2]; %#ok<*NASGU>

% Memory allocation
Ng = length(gammaAll);
CLFCBFLOGS = cell(Ng,1);
xCLFCBF = cell(Ng,1);
legendArray = cell(Ng,1);
for i = 1:Ng
    gamma = gammaAll(i);
    simout = sim(mdl);
    CLFCBFLOGS{i} = logsout2struct(simout.logsout);
    xCLFCBF{i} = CLFCBFLOGS{i}.x;
    legendArray{i} = ['CLF-CBF-QP \gamma = ' num2str(gamma)];
end

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

% CLFCBF trajectories
plt = cell(Ng,1);
for i = 1:Ng
    xCLFCBFi = xCLFCBF{i};
    plt{i} = plot(xCLFCBFi(:,1),xCLFCBFi(:,2),'LineWidth',2);
end

% Update plot
xlabel('x_1','FontSize',14);
ylabel('x_2','FontSize',14);
axis equal;alpha(0.1);
xlim([-20 20]);
ylim([-15 15]);
legend([p1 p2 plt{:}],'LQR Baseline','Unsafe Region',legendArray{:},'FontSize',12,'Location','northwest');