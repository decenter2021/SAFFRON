function [flag] = isOpen(model)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% From Definition 2.3 in [1]
% Links from which it is possible to exit the network
visitedLinks = false(model.Z,1);

% 1. all links with an exit rate allow to leave the network
visitedlinks(model.t0 > eps(0)) = true;

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
            wasUpdated = false;
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
