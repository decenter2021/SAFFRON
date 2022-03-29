function [TTS,RQB] = SFMMetrics(model,xNL)
    % From Section 5.2 in [1]    
    % These criteria are applied to the average of the values over each 
    % cycle interval
    % Take mean 
    xNL_mean = zeros(size(xNL,1),idivide(int16((size(xNL,2)-1)*model.Tsim),int16(model.C)));
    for k = 2:size(xNL,2)
        % Control update frequency is T/C times slower
        if ~rem(int16(k-1),int16(model.C/model.Tsim))
            xNL_mean(:,idivide(int16(k-1),int16(model.C/model.Tsim))) = ...
                 mean(xNL(:,k-(model.C/model.Tsim-1):k),2);
        end
    end
    TTS = model.C*(1/3600)*sum(sum(xNL_mean)); % (veh x h)
    RQB = sum(sum(xNL_mean.^2,2)./model.capacity); % (veh)
end

% References
% [1] Aboudolas, K., Papageorgiou, M., Kosmatopoulos, E., 2009. 
% Store-and-forward based methods for the signal control problem in 
% large-scale congested urban road networks. Transp. Res. C 17 (2), 163-174.