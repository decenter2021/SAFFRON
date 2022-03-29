%% Description - Traffic Network Control
% This script simulates the application of 4 decentralized control methods
% to the signal control problem of congested urban traffic networks:
% 1. DTUC with decentralized configuration psi
% 2. DTUC with decentralized configuration phi
% 3. D2TUC with decentralized configuration psi
% 4. D2TUC with decentralized configuration phi
% It also simulates two centralized methods as a baseline:
% 1. TUC, as detailled in [2]
% 2. TUC - QPC (centralized version of D2TUC) inspired in [2] 
% The implementation is detailled in [1].

%% Initilalize workspace
clear;

%% Import LTI model of Chania network
model = SFMSynthesis("ChaniaUrbanRoadModel");

%% Set deffirent initial conditions and demand if desired
% Below are conditions similar to intermediate demand in [1]
% % Set initial conditions
% rng(1); % Seed for consistency
% model.x0 = 0.5*rand(model.Z,1).*model.capacity;
% % Set demand 
% rng(); % Seed for consistency
% model.d = zeros(model.Z,1);
% model.d(model.inLinks) = 0.1*model.saturation(model.inLinks).*rand(length(model.inLinks),1);
% model.d(model.notInLinks) = 0.01*model.saturation(model.notInLinks).*rand(length(model.notInLinks),1);

%% Modal decomposition as detailed in [1]
% Separation of controllable and uncontrolable modes
ctrbM = ctrb(model.A,model.Bg); % controlability matrix
r = rank(ctrbM);
Z = size(model.A,1);
H = orth(ctrbM);
V = null(H');
W = [H V];
A_hat = W\model.A*W;
Bg_hat = W\model.Bg; 
Bg1_hat = [eye(r) zeros(r,Z-r)]*Bg_hat;
A1_hat = [eye(r) zeros(r,Z-r)]*A_hat*[eye(r);zeros(Z-r,r)];

%% Generate historic actuation as detailed in [1]
gN_DTUC = -(1/model.C)*(Bg1_hat'*Bg1_hat)\Bg1_hat'*[eye(r) zeros(r,Z-r)]/W*model.d;
gN_D2TUC = -(1/model.C)*(model.BG'*model.BG)\model.BG'*model.d;

%% Controller gain synthesis 
% Compute LQR weight matrices as detailled in [1] for DTUC
Q_DTUC = [eye(r) zeros(r,Z-r)]*W'*diag(1./model.capacity)*...
    W*[eye(r) ;zeros(Z-r,r)];
R_DTUC = 0.0001*eye(model.nStages);

% Compute LQR weight matrices as detailled in [1] for D2TUC
Q_D2TUC = diag(1./model.capacity);
R_D2TUC = 0.0001*eye(model.Z);

% Centralized gain computation for DTUC
[K_TUC,P_TUC] = LQROneStepLTI_augmented(A1_hat,Bg1_hat,Q_DTUC,R_DTUC,ones(size(model.E_DTUC_phi)),1e3,1e-5,model.A,model.Bg,r,Z,W);    
% One-step gain computation for DTUC with configuration psi
[K_DTUC_psi,P_DTUC_psi] = LQROneStepLTI_augmented(A1_hat,Bg1_hat,Q_DTUC,R_DTUC,model.E_DTUC_psi,1e3,1e-5,model.A,model.Bg,r,Z,W);
% One-step gain computation for DTUC with configuration phi
[K_DTUC_phi,P_DTUC_phi] = LQROneStepLTI_augmented(A1_hat,Bg1_hat,Q_DTUC,R_DTUC,model.E_DTUC_phi,1e3,1e-5,model.A,model.Bg,r,Z,W);
% Centralized gain computation for D2TUC with configuration phi
[K_D2TUC_C,P_D2TUC_C] = LQRCentralizedLTI(model.A,model.BG,Q_D2TUC,R_D2TUC);
% One-step gain computation for D2TUC with configuration psi
[K_D2TUC_psi,P_D2TUC_psi] = LQROneStepLTI(model.A,model.BG,Q_D2TUC,R_D2TUC,model.E_D2TUC_psi);
% One-step gain computation for D2TUC with configuration phi
[K_D2TUC_phi,P_D2TUC_phi] = LQROneStepLTI(model.A,model.BG,Q_D2TUC,R_D2TUC,model.E_D2TUC_phi);
 
%% Nonlinear simulation
controlStrat = 6; % Number of control strategies to simulate
nDisc = 10; % Number of discrete time steps to simulate
tspan = (0:model.Tsim:nDisc*model.C); % (s) 
xNL = cell(controlStrat,1);
xDisc = cell(controlStrat,1);
dNL = cell(controlStrat,1); 
gNL = cell(controlStrat,1);
uNL = cell(controlStrat,1);

% Variable initialization
for m = 1:controlStrat 
    xNL{m,1} = zeros(model.Z,length(tspan));
    dNL{m,1} = zeros(model.Z,length(tspan)-1);
    gNL{m,1} = zeros(model.nStages,length(0:model.C:tspan(end))-1);
    uNL{m,1} = zeros(model.nStages,length(tspan(end))-1);
end

% Simulate each control strategy
for  m = 1:controlStrat
    xNL{m,1}(:,1) = model.x0;
    for k = 1:length(tspan)-1
        % Control update frequency is T/C times slower if
        if ~rem(int16(k-1),int16(model.C/model.Tsim))
            if k ~= 1
               xDisc{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) = xNL{m,1}(:,k);
            else
                xDisc{m,1}(:,1) = model.x0;
            end
            xD = xDisc{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1);
            switch m 
            case 1 % TUC
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    LQcontrolAction(xD,K_TUC,model,gN_DTUC);
            case 2 % DTUC with configuration psi     
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    LQcontrolAction(xD,K_DTUC_psi,model,gN_DTUC);
            case 3 % DTUC with configuration phi                 
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    LQcontrolAction(xD,K_DTUC_phi,model,gN_DTUC);
            case 4 % TUC - QPC (centralized version of D2TUC) inspired in [2] 
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    QPCcontrolAction(xD,K_D2TUC_C,model,model.S,gN_D2TUC);
            case 5 % DTUC with configuration phi  
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    QPCcontrolAction(xD,K_D2TUC_psi,model,model.S,gN_D2TUC);
            case 6 % DTUC with configuration phi
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    QPCcontrolAction(xD,K_D2TUC_phi,model,model.S,gN_D2TUC);
            end
        end

        % Compute nonlinear control action with upstream gating
        for l = 1:model.Z % Equation (14) of [2]
            if sum(xNL{m,1}(model.T(:,l)~=0,k) >=...
                   model.c*model.capacity(model.T(:,l)~=0))
               uNL{m,1}(l,k) = 0;
            else
               uNL{m,1}(l,k) = min(xNL{m,1}(l,k)/model.Tsim,model.S(l,:)*...
                   gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1)*...
                model.saturation(l)/model.C);
            end
        end
        dNL{m,1}(:,k) = min((model.capacity-...
            xNL{m,1}(:,k)-model.Bu_sim*uNL{m,1}(:,k))/model.Tsim,model.d);
        xNL{m,1}(:,k+1) = xNL{m,1}(:,k)+model.Bu_sim*uNL{m,1}(:,k)+model.Tsim*model.d;
        
        % Catch overspill
        if sum(xNL{m,1}(:,k+1)./model.capacity>1) ~=0
           fprintf("Overspill: method %d | instant k=%d | link %d \n", m,k+1,find(xNL{m,1}(:,k+1)./model.capacity>1));
           break;
        end
      
    end
end

%% Compute performance indices defined in [1,2]
TTS = zeros(controlStrat,1);
RQB = zeros(controlStrat,1);
for  m = 1:controlStrat
    [TTS(m),RQB(m)] = SFMMetrics(model,xNL{m,1});  
end
TTS'
RQB'
    
%% Plots
figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
for m = 1:6
    p = plot(tspan/model.C,sum(abs((V*V')*xNL{m,1}(:,:))));
    p.LineWidth = 3;
end
legend('TUC', 'DTUC|\Psi', 'DTUC|\Phi','D2TUC|Cent.','D2TUC|\Psi','D2TUC|\Phi');
ylabel("$\sum_i|[\mathbf{z_2}]_i|$",'Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;

figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
for m = 1:6
    p = plot(tspan/model.C,sum(abs((H*H')*xNL{m,1}(:,:))));
    p.LineWidth = 3;
end
legend('TUC', 'DTUC|\Psi', 'DTUC|\Phi','D2TUC|Cent.','D2TUC|\Psi','D2TUC|\Phi');
ylabel("$\sum_i|[\mathbf{z_1}]_i|$",'Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;

figure('units','normalized','outerposition',[0 0 1 1]);
hold on;
set(gca,'FontSize',35);
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
for i = [15 40 38 41 16 17]
    r = plot(tspan/model.C,xNL{6,1}(i,:)./model.capacity(i),'LineWidth',3);
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
    r = plot((0:model.Tsim/100:nDisc*model.C-model.Tsim/2)/model.C,gNL{6,1}(i,idivide(int16(((model.Tsim/2:model.Tsim/100:nDisc*model.C))/model.Tsim-1),int16(model.C/model.Tsim))+1),'LineWidth',3);
end
legend('s=29', 's=30', 's=31');
ylabel("$g_s$ (s)",'Interpreter','latex');
xlabel('$k$','Interpreter','latex');
hold off;

%% Auxiliary functions

%% LQcontrolAction - Description
% This function implements a standard linear quadratic feedback control
% action for the traffic network model
% Input:    - x: state
%           - L_LQ: LQ gain matrix
%           - model: struct of variables that characterize the network
%           - gN: historic green times
% Output:   - g: green times for each stage
function g = LQcontrolAction(x,L_LQ,model,gN)
    gpre = -L_LQ*x+gN;
    g = zeros(size(gpre));
    for j = 1:model.J   
        stgsj = (sum(model.nStagesJunction(1:j-1))+1:sum(model.nStagesJunction(1:j-1))+model.nStagesJunction(j));
        a = (gpre(stgsj)-model.gmin(j));
        d = ones(model.nStagesJunction(j),1);
        c = model.C-model.lostTime(j)-model.nStagesJunction(j)*model.gmin(j);
        b = c*ones(model.nStagesJunction(j),1);
        g(stgsj) = knapsack(a,b,c,d)+model.gmin(j);     
    end
end

%% QPCcontrolAction - Description
% This function implements a linear quadratic rogramming feedback control
% action for the traffic network model
% Inspired in the nonlinear QPC control action proposed in [2]
% Input:    - x: state
%           - L_QPC: QPC linear gain matrix
%           - model: struct of variables that characterize the network
%           - S: stage matrix S
%           - gNQPC: historic green times
% Output:   - g: green times for each stage
function g = QPCcontrolAction(x,L_QPC,model,S,gNQPC)
    Gpre = -L_QPC*x+gNQPC;
    gpre = (S'*S)\S'*Gpre;
    g = zeros(size(gpre));
    for j = 1:model.J   
        stgsj = (sum(model.nStagesJunction(1:j-1))+1:sum(model.nStagesJunction(1:j-1))+model.nStagesJunction(j));
        a = (gpre(stgsj)-model.gmin(j));
        d = ones(model.nStagesJunction(j),1);
        c = model.C-model.lostTime(j)-model.nStagesJunction(j)*model.gmin(j);
        b = c*ones(model.nStagesJunction(j),1);
        g(stgsj) = knapsack(a,b,c,d)+model.gmin(j);     
    end
end

%% LQROneStepLTI_augmented - Description
% This function computes the steady-state augmented one-step LQR regulator 
% gain for a window w. Method derived in [1].
% Input:    - A_hat, B_hat
%           - Q, R
%           - E: sparsity pattern
%           - itMax: maximum number of iterations until convergence 
%           - epslInf: minimum relative improvement
%           - A, B
%           - r,Z,W: as defined in [1]
% Output:   - K: nxo steady-state gain matrix
%           - P: nxn steady-state estimation error covariance matrix
% Important notes: 
%           - output gain corresponds to the control law: u(k)=-K(k)*x(k)
% WARNING: Returns Kinf = NaN and Pinf = NaN if convergence could not be reached 
function [K,P] = LQROneStepLTI_augmented(A_hat,B_hat,Q,R,E,itMax,epslInf,A,B,r,Z,W)
% Gain computation
n = size(E,2); % Get value of n from the size of A 
m = size(E,1); % Get value of n from the size of B  
P = Q; % terminal condition
Pprev = NaN;
it = itMax;
while it > 0 % LQ iterations
    K = zeros(m,n);
    S = R+B_hat'*(P)*B_hat;
    for i = 1:n
        L = zeros(n);
        L (i,i) = 1; % Generate matrix L_i
        M = zeros(m);
        for j = 1:m % Gererate matrix M_i
            if E(j,i) ~= 0
                M(j,j) = 1;
            end
        end
        % Compute the ith term of the summation 
        K = K + (eye(m)-M+M*S*M)\(M*(B_hat')*P*A_hat*([eye(r) zeros(r,Z-r)]/W)*L');
    end
    % Update P
    P_ =((W')\[eye(r);zeros(Z-r,r)]) *(P)* ([eye(r) zeros(r,Z-r)]/W);
    Q_ = ((W')\[eye(r);zeros(Z-r,r)]) *(Q)* ([eye(r) zeros(r,Z-r)]/W);
    P = Q_+K'*R*K+...
        (A-B*K)'*P_*(A-B*K);
    P = [eye(r) zeros(r,Z-r)]*W'*P*W*[eye(r);zeros(Z-r,r)];
    % Check convergence
    it = it-1;
    if abs(trace(P)-trace(Pprev))/trace(Pprev) < epslInf
        break; 
    end
    Pprev = P;
    if it == 0
        fprintf("One-step did not converge.\n");
        P = NaN;
        K = NaN;
    end
end 
end

%% References
% [1] Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward 
% based strategies for the signal control problem in large-scale congested 
% urban road networks. Transportation Research Part C: Emerging 
% Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.

% [2] Aboudolas, K., Papageorgiou, M., Kosmatopoulos, E., 2009. 
% Store-and-forward based methods for the signal control problem in 
% large-scale congested urban road networks. Transp. Res. C 17 (2), 163?174.

