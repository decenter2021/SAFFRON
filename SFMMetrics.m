function [TTS,RQB] = SFMMetrics(model,x)
    % From Section 5.2 in [1]
    TTS = model.C*(1/3600)*sum(sum(x)); % (veh x h)
    RBQ = sum(sum(x.^2,2)./model.capacity); % (veh)
end

% References
% [1] Aboudolas, K., Papageorgiou, M., Kosmatopoulos, E., 2009. 
% Store-and-forward based methods for the signal control problem in 
% large-scale congested urban road networks. Transp. Res. C 17 (2), 163-174.