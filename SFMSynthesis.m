% -------------------------------------------------------------------------
% SAFFRON toolbox: https://github.com/decenter2021/SAFFRON
% AUTHORS: Leonardo Pedroso, Pedro Batista, Markos Papageorgiou, and Elias
% Kosmatopoulos 
% LICENSE: MIT License
% If you use SAFFRON, reference the publication below
%   Pedroso, L., Batista, P., Papageorgiou, M. and Kosmatopoulos, E., 2022
%   [not published yet]
% -------------------------------------------------------------------------
%% SFMSynthesis - Description
% This function synthesizes the model of a traffic network from the raw
% data provide in text files. 
% Input:    - folder: path to directory of text files
%               - 'general.txt': general parameters
%               - 'junctions_table.txt': caracteristics of each junction
%               - 'links_table.txt': characteristics of each link
%               - 'stage_matrix.txt': stage matrix
%               - 'stage_table.txt': characteristics of each junction
%               - 'turning_rates_table.txt': turning rates matrix
% (For templates of the text files check "template_*.txt")
% Output:   - model: struct of variables that characterize the network
function model =  SFMSynthesis(folder)
%% Import general parameters
dataGeneral = importdata(folder + "/general.txt");
J = dataGeneral(1,1); % number of junctions (nodes)
Z = dataGeneral(1,2); % number of links
nStages = dataGeneral(1,3); % number of stages
C = dataGeneral(1,4); % control cycle time-step (s)
c = dataGeneral(1,5); % factor c - threshold for definition of a jam
Tsim = dataGeneral(1,6); % simulation time-step (s)

%% Import junctions table
junctionTable = importdata(folder + "/junctions_table.txt");
lostTime = junctionTable(:,1); % time lost at each junction during a control cycle (s)
nStagesJunction = junctionTable(:,2); % number of stages of each junction

%% Import links table
linksTable = importdata(folder + "/links_table.txt");
capacity = linksTable(:,1); % maximum nuber of vehicles in each link
saturation = linksTable(:,2)/(60^2); % saturation flows (ve/s)
lanes = linksTable(:,3); % number of lanes of each link
x0 = linksTable(:,4); % initial condition
d = linksTable(:,5)/(60^2); % demand

%% Import stages matrix
stagesTable = importdata(folder + "/stages_table.txt");
gmin = stagesTable(:,1); % minimum green time for each stage
gN = stagesTable(:,2);% historic green time for each stage

%% Import stage matrix
S = importdata(folder + "/stage_matrix.txt"); % stage matrix (ROW) 

%% Import turning rates matrix
turningRatesTable = importdata(folder + "/turning_rates_table.txt"); 
T = turningRatesTable(:,1:end-1);
t0 = turningRatesTable(:,end);

%% Traffic network graph variables
% Cell to hold the stages of each junction
junctions = cell(J,1); 
count = 1;
% Matrix to hold the junction of origin and destination of each link
links = zeros(Z,2);
% Fill variables 'junctions' and 'links'
for j = 1:J
    junctions{j,1} = count:count+nStagesJunction(j)-1; % stages of junction
    count = count+nStagesJunction(j);
end
for l = 1:Z
    stageOfLink = find(S(l,:)==1);
    for j = 1:J
        if sum(intersect(junctions{j,1},stageOfLink))
            links(l,2) = j;
            break;
        end
    end
end
for l = 1:Z
    stageOfLink = find(S(l,:)==1);
    for j = 1:J
        if sum(intersect(junctions{j,1},stageOfLink))
            links(l,2) = j;
            break;
        end
    end
end

for l = 1:Z
    linkFrom = find(T(l,:)~=0);
    for lF = 1:length(linkFrom)
        links(l,1) = links(linkFrom(lF),2);
    end
end
% Variable to hold the indices of the links that originate outside the network
inLinks = find(links(:,1)==0);
% Variable to hold the indices of the links that do not originate outside the network
notInLinks = find(links(:,1)~=0);

%% Compute LTI state-space system matrices
% Computation according to [1]
A = eye(Z);
% Linear simulation model / control model
% T_sim
Bu_sim = Tsim*(diag(1-t0)*T-eye(Z)); 
BG_sim = (1/C)*Bu_sim*diag(saturation); 
Bg_sim = BG_sim*S; 
% T_ctrl = C
Bu = C*(diag(1-t0)*T-eye(Z)); 
BG = (1/C)*Bu*diag(saturation); 
Bg = BG*S; 
C_z = eye(Z);

%% Compute sparcity patterns
% Sparcity pattern for method DTUC and configuration Psi
E_DTUC_psi = zeros(nStages,Z);
% Sparcity pattern for method DTUC and configuration Phi
E_DTUC_phi = zeros(nStages,Z);

% Compute sparcity patterns 
for stage = 1:nStages % for all stages
    junctionOfStage = 0; % find the corresponding junction
    for j = 1:J
       if sum(junctions{j,1}==stage)
           junctionOfStage = j; % juncrion of stage s found
           break;
       end
    end
    E_DTUC_psi(stage,links(:,1)==junctionOfStage) = 1;
    E_DTUC_psi(stage,links(:,2)==junctionOfStage) = 1;
    E_DTUC_phi(stage,:) = E_DTUC_psi(stage,:);
    junctionOfLinkTowards = links(links(:,2)==junctionOfStage,1);
    junctionOfLinkTowards = junctionOfLinkTowards(junctionOfLinkTowards~=0);
    for junctionTowardsIdx = 1:length(junctionOfLinkTowards)
        E_DTUC_phi(stage,links(:,2)==junctionOfLinkTowards(junctionTowardsIdx)) = 1;
        E_DTUC_phi(stage,links(:,1)==junctionOfLinkTowards(junctionTowardsIdx)) = 1;
    end
    junctionOfLinkFrom = links(links(:,1)==junctionOfStage,2);
    for junctionFromIdx = 1:length(junctionOfLinkFrom)
        E_DTUC_phi(stage,links(:,2)==junctionOfLinkFrom(junctionFromIdx)) = 1;
        E_DTUC_phi(stage,links(:,1)==junctionOfLinkFrom(junctionFromIdx)) = 1;
    end

end

% Sparcity pattern for method D2TUC and configuration Psi
E_D2TUC_psi = zeros(Z,Z);
% Sparcity pattern for method D2TUC and configuration Phi
E_D2TUC_phi = zeros(Z,Z);

% Compute sparcity patterns
for link = 1:Z
    junctionOfLink = links(link,2);  
    E_D2TUC_psi(link,links(:,1)==junctionOfLink) = 1;
    linksTowardsJunction = find(links(:,2)==junctionOfLink); % find the links from juntion of stage
    E_D2TUC_psi(link,linksTowardsJunction) = 1;
    E_D2TUC_phi(link,:) = E_D2TUC_psi(link,:);
    junctionOfLinkTowards = links(linksTowardsJunction,1);
    junctionOfLinkTowards = junctionOfLinkTowards(junctionOfLinkTowards~=0);
    for junctionTowardsIdx = 1:length(junctionOfLinkTowards)
        E_D2TUC_phi(link,links(:,2)==junctionOfLinkTowards(junctionTowardsIdx)) = 1;
        E_D2TUC_phi(link,links(:,1)==junctionOfLinkTowards(junctionTowardsIdx)) = 1;
    end
    junctionOfLinkFrom = links(links(:,1)==junctionOfLink,2);
    for junctionFromIdx = 1:length(junctionOfLinkFrom)
        E_D2TUC_phi(link,links(:,2)==junctionOfLinkFrom(junctionFromIdx)) = 1;
        E_D2TUC_phi(link,links(:,1)==junctionOfLinkFrom(junctionFromIdx)) = 1;
    end
end

%% Fill model struct
model = struct('S',S,...
               'T',T,...
               't0',t0,...
               'J',J,...
               'Z',Z,...
               'nStages',nStages,...
               'C',C,...
               'c',c,...
               'Tsim',Tsim,...
               'lostTime',lostTime,...
               'nStagesJunction',nStagesJunction,...
               'gmin',gmin,...
               'gN',gN,...
               'capacity',capacity,...
               'saturation',saturation,...
               'lanes',lanes,...
               'x0',x0,...
               'junctions',{junctions},...               
               'links',links,...
               'd',d,...
               'inLinks',inLinks,...
               'notInLinks',notInLinks,...
               'A',A,...
               'Bu',Bu,...
               'BG',BG,...
               'Bg',Bg,...
               'Bu_sim',Bu_sim,...
               'BG_sim',BG_sim,...
               'Bg_sim',Bg_sim,...
               'C_z',C_z,...
               'E_DTUC_psi',E_DTUC_psi,...
               'E_DTUC_phi',E_DTUC_phi,...
               'E_D2TUC_psi',E_D2TUC_psi,...
               'E_D2TUC_phi',E_D2TUC_phi);
           
%% Save synthesized model
save(sprintf("%s/data.mat",folder),'junctionTable','linksTable','S','turningRatesTable','t0','T',...
    'J','Z','nStages','C','c','Tsim','lostTime','nStagesJunction','gmin','gN','capacity','saturation',...
    'lanes','x0','d','junctions','links','inLinks','notInLinks','A','Bu','BG','Bg','Bu_sim',...
    'BG_sim','Bg_sim','C_z','E_DTUC_psi','E_DTUC_phi','E_D2TUC_psi','E_D2TUC_phi','dataGeneral','stagesTable');
end
