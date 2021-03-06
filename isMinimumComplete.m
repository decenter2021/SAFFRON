% -------------------------------------------------------------------------
% SAFFRON toolbox: https://github.com/decenter2021/SAFFRON
% AUTHORS: Leonardo Pedroso, Pedro Batista, Markos Papageorgiou, and Elias
% Kosmatopoulos 
% LICENSE: MIT License
% If you use SAFFRON, reference the publication below
%   Pedroso, L., Batista, P., Papageorgiou, M. and Kosmatopoulos, E., 2022
%   [not published yet]
% -------------------------------------------------------------------------
%% isMinimumComplete - Description 
% This function checks whether a traffic network has a minimum complete
% stage strategy according to [1]
% Input:    - model: struct of variables that characterize the network
% Output:   - flag: boolean that indicates if the network has a minimum 
% complete stage strategy
function [flag] = isMinimumComplete(model)

% From Definition 2.3 in [1]
% Condition 1
for j = 1:size(model.S,2)
    if (sum(model.S(:,j)) < 1)
        flag = false;
        return;
    end
end

% Condition 2
for i = 1:size(model.S,1)
    if (sum(model.S(i,:)) < 1)
        flag = false;
        return;
    end
end


for junction = 1:model.J
    stages = model.junctions{junction};
    % Condition 3
    if sum(sum(model.S(model.links(:,2) ~= junction,stages))) > 0
        flag = false;
        return;
    end
    
    % Condition 4
    if rank(model.S(model.links(:,2)==junction,stages)) < length(stages)
        flag = false;
        return;
    end
end
flag = true;
end

%% References
% [1] Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward 
% based strategies for the signal control problem in large-scale congested 
% urban road networks. Transportation Research Part C: Emerging 
% Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.
