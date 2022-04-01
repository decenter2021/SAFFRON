% -------------------------------------------------------------------------
% SAFFRON toolbox: https://github.com/decenter2021/SAFFRON
% AUTHORS: Leonardo Pedroso, Pedro Batista, Markos Papageorgiou, and Elias
% Kosmatopoulos 
% LICENSE: MIT License
% If you use SAFFRON, reference the publication below
%   Pedroso, L., Batista, P., Papageorgiou, M. and Kosmatopoulos, E., 2022
%   [not published yet]
% -------------------------------------------------------------------------
%% isOpen - Description 
% This function checks whether a traffic network is open according to [1]
% Input:    - model: struct of variables that characterize the network
% Output:   - flag: boolean that indicates if the network is open according
function [flag] = isOpen(model)
% From Definition 2.3 in [1]
% Links from which it is possible to exit the network
visitedLinks = false(model.Z,1);

% 1. all links with an exit rate allow to leave the network
visitedLinks(model.t0 > eps(0)) = true;

% 2. There are junctions with links (dashed) towards outside the network
% The links z with ability to turn to those links do not sum up to 1 in the
% corresponding column of T, i.e., in these links T(:,z) ~= 1
visitedLinks(sum(model.T,1) < 1-eps(1)) = true;

% Run until no changes are observed
while true
    wasUpdated = false;
    % Check if a vehicle can exit or every link for every link 
    for z = 1:model.Z
        % If link was already found to lead to an exit
        if visitedLinks(z) == true
            continue;
        end

        % Links towads which link z can turn
        if sum(visitedLinks(model.T(:,z)>eps(0))) > 0
            visitedLinks(z) = true;
            wasUpdated = true;
        end
    end
    
    % Check if iterative procedure finished
    if sum(~visitedLinks)>0
        if wasUpdated
            continue;
        else
            flag = false;
            return;
        end
    else
        flag = true;
        return;
    end
    
end

%% References
% [1] Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward 
% based strategies for the signal control problem in large-scale congested 
% urban road networks. Transportation Research Part C: Emerging 
% Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.
