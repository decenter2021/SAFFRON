% -------------------------------------------------------------------------
% SAFFRON toolbox: https://github.com/decenter2021/SAFFRON
% AUTHORS: Leonardo Pedroso, Pedro Batista, Markos Papageorgiou, and Elias
% Kosmatopoulos 
% LICENSE: MIT License
% If you use SAFFRON, reference the publication below
% 
% L. Pedroso, P. Batista, M. Papageorgiou and E. Kosmatopoulos, 
% 'SAFFRON: Store-And-Forward model toolbox For urban ROad Network signal 
% control in MATLAB', 2022 IEEE 25th International Conference on 
% Intelligent Transportation Systems (ITSC), pp. 3698-3703, 2022. 
% doi: 10.1109/ITSC55140.2022.9922508.
%
% -------------------------------------------------------------------------

%% Description 
% Feedback-feedforward Signal Control with Exogenous Demand Estimation in 
% Congested Urban Road Networks
% Simulation results of:
%
% L. Pedroso, P. Batista, and M. Papageorgiou and E. Kosmatopoulos, 
% 'Feedback-feedforward Signal Control with Exogenous Demand Estimation in 
% Congested Urban Road Networks' (under review)
%

%% Initilalize workspace
clear;

%% Import LTI model of Chania network
model = SFMSynthesis("ChaniaUrbanRoadModel");
% Change cycle time to 100s (so that the estimator smapling time is an integer multiple)
model.C = 100;
model.Bu = model.Bu*(100/90);
model.BG = model.Bu*(100/90);
model.Bg = model.Bg*(100/90);

%% Set deffirent initial conditions and demand if desired
% Set initial conditions
rng(0); % Seed for consistency
model.x0 = 0.05*(0.2*rand(model.Z,1)+0.8).*model.capacity;
% Set demand 
model.d = zeros(model.Z,1);
model.d(model.inLinks) = 0.1*model.saturation(model.inLinks).*(0.2*rand(length(model.inLinks),1)+0.5);
model.d(model.notInLinks) = 0.005*model.saturation(model.notInLinks).*(0.2*rand(length(model.notInLinks),1)+0.8);

%% Set demand variation model 
d_model_A = (rand(size(model.d))*0.5+0.25).*model.d; % Amplitude is 0.25-0.75 of nominal demand
d_model_phase = rand(size(model.d))*2*pi; 
d_model_T = (rand(size(model.d))*90+30)*60; % Period between 30 min and 2h (in seconds)
d_model_dt_fall = 2*60*60; 
d_model_spike_N_links = 2;
d_model_spike_links = [21 51];%round((model.Z-1)*rand(1,d_model_spike_N_links))+1;
d_model_spike_mag_of_sat = [0.35 0.35];
d_model_spike_t = (rand(d_model_spike_N_links,1)+1.5)*60*60;
d_model_spike_dt = 1.5*60*60;

%% Modal decomposition
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

%% Controller gain synthesis 
% Compute LQR weight matrices
Q_TUC = [eye(r) zeros(r,Z-r)]*W'*diag(1./model.capacity)*...
    W*[eye(r) ;zeros(Z-r,r)];
R_TUC = 0.0001*eye(model.nStages);

% Centralized gain computation for DTUC
[P,K1,~] = idare(A1_hat,Bg1_hat,Q_TUC,R_TUC);
K = K1*([eye(r) zeros(r,Z-r)]/W);

Acl = eye(r)-Bg1_hat*K1;
Kd1 = (Bg1_hat'*P*Bg1_hat+R_TUC)\Bg1_hat'*inv(eye(r)-Acl')*P;
Kd = Kd1*([eye(r) zeros(r,Z-r)]/W);
gN_TUC = - model.C*Kd*model.d;

%% Kalman filter synthesis
% Estimation
E = 20; % E = 20 s  
T_est_Tsim = round(E/model.Tsim); 
% Estimation w/ d
A_filt = [1 T_est_Tsim*model.Tsim; 0 1];
C_filt = [1 0];
R_filt = cell(model.Z,1);
Q_filt = cell(model.Z,1);
K_filt = cell(model.Z,1);
for j = 1:model.Z
    R_filt{j,1} = (0.05*model.capacity(j)/4)^2; % Occupancy measuremnt 
    Q_filt{j,1} = diag([...
        (model.saturation(j)*model.Tsim*T_est_Tsim/10)^2 ...
        (model.saturation(j)*model.Tsim*T_est_Tsim/1e3)^2]); % Dynamics process noise  
    [K_filt{j,1},~] = kalmanCentralizedLTI(A_filt,C_filt,Q_filt{j,1},R_filt{j,1});
end
% Estimation wo/ d
A_filt_wod = 1;
C_filt_wod = 1;
R_filt_wod = cell(model.Z,1);
Q_filt_wod = cell(model.Z,1);
K_filt_wod = cell(model.Z,1);
w_wod_var = ((1/60)*T_est_Tsim*model.Tsim).^2;
for j = 1:model.Z
    R_filt_wod{j,1} = (0.05*model.capacity(j)/4)^2; % Occupancy measuremnt 
    Q_filt_wod{j,1} = (model.saturation(j)*model.Tsim*T_est_Tsim/10)^2; % Dynamics process noise  
    [K_filt_wod{j,1},~] = kalmanCentralizedLTI(A_filt_wod,C_filt_wod,Q_filt_wod{j,1},R_filt_wod{j,1});
end

%% Nonlinear simulation
controlStrat = 4; % Number of control strategies to simulate
nDisc = (8*60*60-d_model_dt_fall)/model.C; % Number of discrete control time steps to simulate
tspan = (0:model.Tsim:nDisc*model.C+d_model_dt_fall); % (s) 
xNL = cell(controlStrat,1);
xDisc = cell(controlStrat,1);
dNL = cell(controlStrat,1); 
gNL = cell(controlStrat,1);
uNL = cell(controlStrat,1);
uNL_hat = cell(controlStrat,1);
% Demand
d = zeros(size(model.d,1),length(tspan));
blocked_veh_links = zeros(model.Z,controlStrat);
blocked_veh = cell(controlStrat,1);
% Estimation
xhat = cell(controlStrat,1);
uhat = cell(controlStrat,1);
dhat = cell(controlStrat,1);
y = cell(controlStrat,1);
y_noise_stdmvn = mvnrnd(zeros(1,model.Z),eye(model.Z),ceil((length(tspan)-1)/T_est_Tsim))';
y_switch_std = mvnrnd(zeros(1,model.Z),eye(model.Z),length(tspan)-1)';
y_switch_std = bandpass(y_switch_std',[1/(model.C) 1/(model.C/2)],1/model.Tsim)';
y_switch_std = y_switch_std(:,1:T_est_Tsim:(length(tspan)-1));
q_in_noise_stdmvn = mvnrnd(zeros(1,model.Z),eye(model.Z),ceil((length(tspan)-1)/T_est_Tsim))';
q_out_noise_stdmvn = mvnrnd(zeros(1,model.Z),eye(model.Z),ceil((length(tspan)-1)/T_est_Tsim))';

% Variable initialization
for m = 1:controlStrat 
    xNL{m,1} = zeros(model.Z,length(tspan));
    dNL{m,1} = zeros(model.Z,length(tspan)-1);
    gNL{m,1} = zeros(model.nStages,length(0:model.C:tspan(end))-1);
    uNL{m,1} = zeros(model.nStages,length(tspan(end))-1);
    blocked_veh{m,1} = zeros(model.Z,length(tspan));
    if m == 3 || m == 4
        xhat{m,1} = zeros(model.Z,length(tspan));
        uhat{m,1} = zeros(model.Z,length(tspan));
        dhat{m,1} = zeros(model.Z,length(tspan));
        y{m,1} = zeros(model.Z,length(tspan));
    end
end

% Simulate each control strategy
for  m = 1:controlStrat
    % Initialize demand and initial state
    xNL{m,1}(:,1) = model.x0;
    d(:,1) = model.d + d_model_A.*sin(d_model_phase);
    for k = 1:length(tspan)-1
        %% Estimation
        if ~rem(int16(k-1),T_est_Tsim) && (m == 3 || m == 4)
            k_disc = idivide(int16(k-1),int16(T_est_Tsim))+1;
            k_disc_ctrl = idivide(int16(k-2),int16(model.C/model.Tsim))+1;
            % Ground truth variables
            x_gt = xNL{m,1}(:,k);
            if k~= 1
                Bu_sim_in = model.Bu_sim;
                Bu_sim_in(Bu_sim_in < 0) = 0; 
                Bu_sim_out = -model.Bu_sim;
                Bu_sim_out(Bu_sim_out < 0) = 0;
                q_in_gt = sum(Bu_sim_in*uNL{m,1}(:,k-T_est_Tsim:k-1),2)/(T_est_Tsim*model.Tsim);
                q_out_gt = sum(Bu_sim_out*uNL{m,1}(:,k-T_est_Tsim:k-1),2)/(T_est_Tsim*model.Tsim);
                delta_q_gt = (xNL{m,1}(:,k)-xNL{m,1}(:,k-T_est_Tsim)-dNL{m,1}(:,k-1)*T_est_Tsim*model.Tsim)/(T_est_Tsim*model.Tsim);
            end
            % Occupancy and flow measurment
            y{m,1}(:,k_disc) = 0.05*x_gt.*y_noise_stdmvn(:,k_disc) + x_gt+ 0.4*x_gt.*y_switch_std(:,k_disc);
            if k ~= 1
                delta_qT = delta_q_gt*T_est_Tsim*model.Tsim + T_est_Tsim*model.Tsim*0.2*...
                    (q_in_gt.*q_in_noise_stdmvn(:,k_disc)-q_out_gt.*q_out_noise_stdmvn(:,k_disc));
            end
            % Estimate qs
            if k~= 1
                % Predicted input by the controller with estimated nonlinearity
                xhat_sat = sat(xhat{m,1}(:,k_disc-1),0, model.capacity);
                for l = 1:model.Z % Equation (14) of [2]
                    if sum(xhat_sat(model.T(:,l)~=0) >=...
                           model.c*model.capacity(model.T(:,l)~=0))
                       uhat{m,1}(l,k_disc-1) = 0;
                    else
                       uhat{m,1}(l,k_disc-1) = min(xhat_sat(l)/model.Tsim,model.S(l,:)*...
                           gNL{m,1}(:,k_disc_ctrl)*model.saturation(l)/model.C);
                    end   
                end
                %Estimate
                q_in_hat = sum(Bu_sim_in*uhat{m,1}(:,k_disc-1),2)*T_est_Tsim/(T_est_Tsim*model.Tsim);
                q_out_hat = sum(Bu_sim_out*uhat{m,1}(:,k_disc-1),2)*T_est_Tsim/(T_est_Tsim*model.Tsim);
                delta_qT_hat = (q_in_hat-q_out_hat)*T_est_Tsim*model.Tsim;
            end
            switch m
            case 3 % TUC est. d, est. feed
                % Compute estimate (without measured qs)
                if k~= 1
                    x_hat_plus = xhat{m,1}(:,k_disc-1) + T_est_Tsim*model.Tsim*dhat{m,1}(:,k_disc-1) + delta_qT_hat;
                    for j = 1:model.Z
                        xhat{m,1}(j,k_disc) = x_hat_plus(j) + K_filt{j,1}(1,:)*(y{m,1}(j,k_disc)-x_hat_plus(j));
                        dhat{m,1}(j,k_disc) = dhat{m,1}(j,k_disc-1) + K_filt{j,1}(2,:)*(y{m,1}(j,k_disc)-x_hat_plus(j));
                    end
                else
                    xhat{m,1}(:,k_disc) = y{m,1}(:,k_disc);
                    dhat{m,1}(:,k_disc) = zeros(model.Z,1);
                end
            case 4 % TUC nom. d, est. feed
                % Real-time occupancy and exogenous demand estimation
                % Compute estimate
                if k~= 1
                    x_hat_plus = xhat{m,1}(:,k_disc-1) + T_est_Tsim*model.Tsim*model.d + delta_qT;
                    for j = 1:model.Z
                        xhat{m,1}(j,k_disc) = x_hat_plus(j) + K_filt_wod{j,1}(1,:)*(y{m,1}(j,k_disc)-x_hat_plus(j));
                    end
                else
                    xhat{m,1}(:,k_disc) = y{m,1}(:,k_disc);
                end
            end
        end

        %% Control
        % Control update frequency is T/C times slower if
        if ~rem(int16(k-1),int16(model.C/model.Tsim))
            if k ~= 1
                xDisc{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) = xNL{m,1}(:,k);
            else
                xDisc{m,1}(:,1) = model.x0;
            end
            xD = xDisc{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1);
            switch m 
                case 1 % TUC nom. d, perf. feed
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    LQcontrolAction(xD,K,model,gN_TUC);                
                case 2 % TUC perf. d, perf. feed                 
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    LQcontrolAction(xD,K,model,-model.C*Kd*d(:,k));                              
                case 3 % TUC est. d, est. feed
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    LQcontrolAction(sat(xhat{m,1}(:,k_disc),0,model.capacity),K,model,-model.C*Kd*dhat{m,1}(:,k_disc));    
                case 4 % TUC nom. d, est. feed
                gNL{m,1}(:,idivide(int16(k-1),int16(model.C/model.Tsim))+1) =...
                    LQcontrolAction(sat(xhat{m,1}(:,k_disc),0,model.capacity),K,model,gN_TUC);
            end
        end

        %% Nonlinear simulation
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
        
        for i = 1:model.Z
            dif = d(i,k)*model.Tsim - (0.99*model.capacity(i)-xNL{m,1}(i,k)-model.Bu_sim(i,:)*uNL{m,1}(:,k));
            if dif > 0 % Veh emand is greater than road availability
                dNL{m,1}(i,k) = d(i,k) - dif/model.Tsim;
                blocked_veh_links(i,m) = blocked_veh_links(i,m) + dif;
                blocked_veh{m,1}(i,k) = dif;
            else % Available veh demand is -dif
                serve_blocked = min([blocked_veh_links(i,m) -dif]);
                blocked_veh_links(i,m) = blocked_veh_links(i,m) - serve_blocked;
                dNL{m,1}(i,k) = d(i,k) + serve_blocked/model.Tsim;
                blocked_veh{m,1}(i,k) = 0;
            end
        end

        % Dynamics
        xNL{m,1}(:,k+1) = xNL{m,1}(:,k)+model.Bu_sim*uNL{m,1}(:,k)+model.Tsim*dNL{m,1}(:,k);

        % Catch overspill
        if sum(xNL{m,1}(:,k+1)./model.capacity>1) ~=0
           fprintf("Overspill: method %d | instant k=%d | link %d \n", m,k+1,find(xNL{m,1}(:,k+1)./model.capacity>1+eps(1)));
           break;
        end
        
        %% Exogenous demand generation
        % Next intant demand 
        d(:,k+1) = model.d + d_model_A.*sin(k*model.Tsim*2*pi./d_model_T +d_model_phase);
        for i = 1:d_model_spike_N_links
            if tspan(k+1) >= d_model_spike_t(i) && tspan(k+1) <= d_model_spike_t(i) + d_model_spike_dt
                d(d_model_spike_links(i),k+1) = model.saturation(d_model_spike_links(i))*d_model_spike_mag_of_sat(i);
            end
        end
        if tspan(k+1) > tspan(end)-d_model_dt_fall
            d(:,k+1) = d(:,k+1)*exp(-(tspan(k+1)-(tspan(end)-d_model_dt_fall))/(d_model_dt_fall/4));
        end
             
    end
end

%% Compute performance indices defined in [1,2]
TTS = zeros(controlStrat,1);
RQB = zeros(controlStrat,1);
overall_blocked_veh = zeros(controlStrat,1);
for  m = 1:controlStrat
%     [TTS(m),RQB(m)] = SFMMetrics(model,xNL{m,1});  
    [TTS(m),RQB(m)] = SFMMetricsWblocked(model,xNL{m,1},blocked_veh{m,1});
    overall_blocked_veh(m) = sum(sum(blocked_veh{m,1}));
end
TTS'
RQB'
sum(blocked_veh_links)
overall_blocked_veh'

%% Plots

% Demand
figure('Position',3*[0 0 192 144]);
hold on;
grid on;
box on;
set(gca,'FontSize',20);
set(gca, 'Layer', 'top');
set(gca,'TickLabelInterpreter','latex');
for i = [12 33 39 d_model_spike_links]
    plot(tspan/(60*60),d(i,:),'LineWidth',3);   
    %plot(tspan(1:end-1)/(60*60),dNL{1,1}(i,:),'--k','LineWidth',1);
end
for i = [12 33 39 d_model_spike_links]
    plot(tspan/(60*60),model.d(i)*ones(size(tspan/(60*60))),'--k','LineWidth',2);
end
legend({'$z=12$','$z=33$','$z=39$','$z=21$','$z=51$','$e_z^{\mathrm{\,hist}}$'}, 'Interpreter','latex');
ylabel("$e_z \quad [\mathrm{veh}\;\mathrm{s}^{-1}]$",'Interpreter','latex');
xlabel('hours','Interpreter','latex');
hold off;
% saveas(gcf,'./d_sc1.eps','epsc');

% Estimation occupancy of link 21
m = 3;
link = 21;
figure('Position',3*[0 0 192 144]);
hold on;
grid on;
box on;
set(gca,'FontSize',20);
set(gca, 'Layer', 'top');
set(gca,'TickLabelInterpreter','latex');
for i = link %d_model_spike_links(1) %[15 40 38 41 16 17]
    plot(tspan(1:T_est_Tsim:end)/(60*60),y{m,1}(i,1:length(1:T_est_Tsim:end))./model.capacity(i),'--k','LineWidth',1);
    plot(tspan(1:T_est_Tsim:end)/(60*60),xhat{m,1}(i,1:length(1:T_est_Tsim:end))./model.capacity(i),'LineWidth',2);
    plot(tspan/(60*60),xNL{m,1}(i,:)./model.capacity(i),'LineWidth',2);
end
legend({'$y_z$','$\hat{x}_z$','$x_z$'}, 'Interpreter','latex');
ylabel("$\hat{x}_z/x_{z,max} \quad (z = 21)$",'Interpreter','latex');
xlabel('hours','Interpreter','latex');
hold off;
% saveas(gcf,'./x_hat_sc1.eps','epsc');

% Estimation exogenous demand of link 21
m = 3;
link = 21;
figure('Position',3*[0 0 192 144]);
hold on;
grid on;
box on;
set(gca,'FontSize',20);
set(gca, 'Layer', 'top');
set(gca,'TickLabelInterpreter','latex');
for i = link %d_model_spike_links(1) %[15 40 38 41 16 17]
    plot(tspan(1:T_est_Tsim:end)/(60*60),dhat{m,1}(i,1:length(1:T_est_Tsim:end)),'LineWidth',3);
    %plot(tspan/(60*60),dhat{m,1}(i,:),'LineWidth',3);
    plot(tspan(1:end-1)/(60*60),dNL{m,1}(i,:),'LineWidth',3);
end
legend({'$\hat{e}_z$','$e_z$'}, 'Interpreter','latex');
ylabel("$e_z \quad [\mathrm{veh}\;\mathrm{s}^{-1}] \quad (z = 21)$",'Interpreter','latex');
xlabel('hours','Interpreter','latex');
hold off;
% saveas(gcf,'./d_hat_sc1.eps','epsc');

% Occupancy
figure('Position',3*[0 0 192 144]);
hold on;
grid on;
box on;
set(gca,'FontSize',20);
set(gca, 'Layer', 'top');
set(gca,'TickLabelInterpreter','latex');
for m = 1:controlStrat
    p = plot(tspan/(60*60),sum(abs(xNL{m,1}(:,:))));
    p.LineWidth = 3;
end
legend({'TUC ideal', 'TUC-FF ideal', 'TUC-FF','TUC'},'Interpreter','latex');
%legend({'TUC nom. $d$, perf. $x$', 'TUC perf. $d$, perf. $x$', 'TUC est. $d$, est. $x$','TUC nom. $d$, est. $x$'},'Interpreter','latex');
ylabel("$\sum_i [\mathbf{x}]_i\quad \mathrm{veh}$",'Interpreter','latex');
xlabel('hours','Interpreter','latex');
hold off;
% saveas(gcf,'./sum_x_sc1.eps','epsc');


% Green-times junction 8
figure('Position',3*[0 0 192 144]);
hold on;
grid on;
box on;
set(gca,'FontSize',20);
set(gca, 'Layer', 'top');
set(gca,'TickLabelInterpreter','latex');
colors = [0, 0.4470, 0.7410;...
          0.8500, 0.3250, 0.0980;...
          0.9290, 0.6940, 0.1250];
stages_plt = model.junctions{8};
for i = 1:length(stages_plt)
    for m = 3:4
        r = plot((0:model.Tsim/100:nDisc*model.C+d_model_dt_fall-model.Tsim/2)/(60*60),gNL{m,1}(stages_plt(i),idivide(int16(((model.Tsim/2:model.Tsim/100:nDisc*model.C+d_model_dt_fall))/model.Tsim-1),int16(model.C/model.Tsim))+1),'LineWidth',3);
        r.Color = colors(i,:);
        if m == 4, r.LineStyle = '--'; end
    end
end
legend('$s=19$ (TUC-FF)', '$s=19$ (TUC)', '$s=20$ (TUC-FF)', '$s=20$ (TUC)','Interpreter','latex');
ylabel("$g_s \quad (\mathrm{s})$",'Interpreter','latex');
xlabel('hours','Interpreter','latex');
hold off;
% saveas(gcf,'./g8_sc1.eps','epsc');

% Green-times junction 6
figure('Position',3*[0 0 192 144]);
hold on;
grid on;
box on;
set(gca,'FontSize',20);
set(gca, 'Layer', 'top');
set(gca,'TickLabelInterpreter','latex');
colors = [0, 0.4470, 0.7410;...
          0.8500, 0.3250, 0.0980;...
          0.9290, 0.6940, 0.1250];
stages_plt = model.junctions{6};
for i = 1:length(stages_plt)
for m = 3:4
        r = plot((0:model.Tsim/100:nDisc*model.C+d_model_dt_fall-model.Tsim/2)/(60*60),gNL{m,1}(stages_plt(i),idivide(int16(((model.Tsim/2:model.Tsim/100:nDisc*model.C+d_model_dt_fall))/model.Tsim-1),int16(model.C/model.Tsim))+1),'LineWidth',2);
        %if m == 4, r.Color = colors(i,:)+0.4*(1-colors(i,:)); end
        r.Color = colors(i,:);
        if m == 3, r.Color = [colors(i,:) 0.5] ; end
        if m == 4, r.LineStyle = '--'; end
    end
end
legend('$s=14$ (TUC-FF)', '$s=14$ (TUC)', '$s=15$ (TUC-FF)', '$s=15$ (TUC)', '$s=16$ (TUC-FF)', '$s=16$ (TUC)','Interpreter','latex');
ylabel("$g_s \quad (\mathrm{s})$",'Interpreter','latex');
xlabel('hours','Interpreter','latex');
hold off;
% saveas(gcf,'./g6_sc1.eps','epsc');

%% Auxiliary functions

%% Saturatiuon function
function x = sat(x,lb,ub)
    if length(lb) == 1
        x(x<lb) = lb;
    else
        x(x<lb) = lb(x<lb);
    end
    if length(ub) > 1
        x(x>ub) = ub(x>ub);
    else
        x(x>ub) = ub;
    end
end

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
        a = (gpre(stgsj)-model.gmin(stgsj));
        d = ones(model.nStagesJunction(j),1);
        c = model.C-model.lostTime(j)-sum(model.gmin(stgsj));
        b = c*ones(model.nStagesJunction(j),1);
        g(stgsj) = knapsack(a,b,c,d)+model.gmin(stgsj);     
    end
end

function [TTS,RQB] = SFMMetricsWblocked(model,xNL,blocked_veh)
    % From Section 5.2 in [1]    
    % These criteria are applied to the average of the values over each 
    % cycle interval
    % Take mean 
    xNL_mean = zeros(size(xNL,1),idivide(int16((size(xNL,2)-1)*model.Tsim),int16(model.C)));
    for k = 2:size(xNL,2)
        % Control update frequency is T/C times slower
        if ~rem(int16(k-1),int16(model.C/model.Tsim))
            xNL_mean(:,idivide(int16(k-1),int16(model.C/model.Tsim))) = ...
                 mean(xNL(:,k-(model.C/model.Tsim-1):k)+ blocked_veh(:,k-(model.C/model.Tsim-1):k),2);
        end
    end
    TTS = model.C*(1/3600)*sum(sum(xNL_mean)); % (veh x h)
    RQB = sum(sum(xNL_mean.^2,2)./model.capacity); % (veh)
end

%% References
% [1] Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward 
% based strategies for the signal control problem in large-scale congested 
% urban road networks. Transportation Research Part C: Emerging 
% Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.

% [2] Aboudolas, K., Papageorgiou, M., Kosmatopoulos, E., 2009. 
% Store-and-forward based methods for the signal control problem in 
% large-scale congested urban road networks. Transp. Res. C 17 (2), 163?174.

