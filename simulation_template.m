%% Description - Template Traffic Network Control
% This script is a template for the implementation of a tarffic resposive
% control solution using the store-and-forward model using SAFFRON toolbox

% An example of the implemenattion of the simulations in [2] is available
% in Examples/PedrosoBatista2021

%% Initilalize workspace
clear;

%% Import LTI model of urban raod network
% Import Chania network
model = SFMSynthesis("ChaniaUrbanRoadModel");

%% Set different initial conditions and demand if desired
% Chania urban road model has initial condictions and demands for a
% scenario. These can be used as a benchmark. To test different initial
% conditions and demand scenarios simply make modifications to model.x0 and
% model.d

% For example:
rng(1); % Seed for consistency
% Set initial conditions
model.x0 = 0.5*rand(model.Z,1).*model.capacity;
% Set demand 
model.d = zeros(model.Z,1);
model.d(model.inLinks) = 0.1*model.saturation(model.inLinks).*rand(length(model.inLinks),1);
model.d(model.notInLinks) = 0.01*model.saturation(model.notInLinks).*rand(length(model.notInLinks),1);

%% Assess controllability of the models
% From Definition 2.1 in [2]: open & finite -> Feasible traffic network
if ~isOpen(model)
    fprintf("Unfeasible traffic network.");
    return;
end
% From Definition 2.3 in [2]:
if ~isMinimumComplete(model)
    fprintf("Non minimum complete stage strategy.");
    return;
end

% Conclusions about controlability of the store-and-forward models can be 
% taken from Propositions 3.1 and 4.1 in [2]

%% Controller synthesis 
% Synthesize a control law from the parameters of the store-and-forward
% model. For the example of the synthesis of the methods proposed in [2] 
% and TUC [3,4] see Examples/PedrosoBatista2021


%% Nonlinear simulation
nDisc = 10; % Number of discrete time steps to simulate
tspan = (0:model.Tsim:nDisc*model.C); % (s) 

% Variable initialization
xNL = zeros(model.Z,length(tspan));
dNL = zeros(model.Z,length(tspan)-1);
gNL = zeros(model.nStages,length(0:model.C:tspan(end))-1);
uNL = zeros(model.nStages,length(tspan(end))-1);

% Simulate control strategy
xNL(:,1) = model.x0;
for k = 1:length(tspan)-1
    
    % Control update frequency is T/C times slower
    if ~rem(int16(k-1),int16(model.C/model.Tsim))
        % Implement Control law
        gNL(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) = ...
         % f(xNL(:,k),...);
    end
    
    % Compute nonlinear control action with upstream gating
    for l = 1:model.Z % Equation (14) of [3]
        if sum(xNL(model.T(:,l)~=0,k) >= model.c*model.capacity(model.T(:,l)~=0))
            uNL(l,k) = 0;
        else
            uNL(l,k) = min(xNL(l,k)/model.Tsim,model.S(l,:)*...
                gNL(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1)*...
                model.saturation(l)/model.C);
        end
    end
    
    % Vehicles cannot enter links inside the network or from the outside if
    % they are full
    dNL(:,k) = min((model.capacity-...
        xNL(:,k)-model.Bu_sim*uNL(:,k))/model.Tsim,model.d);
    
    % Vehicle dynamics
    xNL(:,k+1) = xNL(:,k)+model.Bu_sim*uNL(:,k)+model.Tsim*model.d;

    % Catch overspill
    if sum(xNL(:,k+1)./model.capacity>1) ~= 0
       fprintf("Overspill: instant %d | link %d.\n",k+1,find(xNL(:,k+1)./model.capacity>1));
       break;
    end
end

%% Compute performance indices defined in [1,2]
[TTS,RQB] = SFMMetrics(model,xNL);  
fprintf("TTS: %g | RQB: %g\n",TTS,RQB);
    
%% Plot results
figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
for i = [15 40 38 41 16 17]
    r = plot(tspan/model.C,xNL(i,:)./model.capacity(i),'LineWidth',3);
end
legend('z=15', 'z=40', 'z=38','z=41','z=16','z=17');
ylabel("$x_z/x_{z,max}$",'Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;

figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
for i = [29 30 31]
    r = plot((0:model.Tsim/100:nDisc*model.C-model.Tsim/2)/model.C,gNL(i,idivide(int16(((model.Tsim/2:model.Tsim/100:nDisc*model.C))/model.Tsim-1),int16(model.C/model.Tsim))+1),'LineWidth',3);
end
legend('s=29', 's=30', 's=31');
ylabel("$g_s$ (s)",'Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;

%% References
% [1] Not published yet

% [2] Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward 
% based strategies for the signal control problem in large-scale congested 
% urban road networks. Transportation Research Part C: Emerging 
% Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.

% [3] Aboudolas, K., Papageorgiou, M., Kosmatopoulos, E., 2009. 
% Store-and-forward based methods for the signal control problem in 
% large-scale congested urban road networks. Transp. Res. C 17 (2), 163?174.

% [4] Diakaki, C., 1999. Integrated control of traffic flow in corridor 
% networks. Ph. D. Thesis.
