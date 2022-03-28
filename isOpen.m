function [flag] = isOpen(model)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% From Definition 2.3 in [1]
% Links from which it is possible to exit the network
visitedLinks = false(model.Z,1);
% All links with an exit rate allow to leave the network
visitedlinks(model.t0 > 0) = true;

% Check if a vehicle can exit or every link for every link 
for link = 1:model.Z
    % If link was already found to lead to an exit
    if visitedLinks(link) == true
        continue;
    end
    
    if model.t0(link) > 0 
        visitedLinks(link) = true;
        continue;
    end
        
        
end

%% References
% [1] Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward 
% based strategies for the signal control problem in large-scale congested 
% urban road networks. Transportation Research Part C: Emerging 
% Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.
