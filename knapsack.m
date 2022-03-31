%% knapsack - Description
% This function solves the knapsack problem 
% Implementation of algorithm in [1]
% Input:    - a,b,c,d as defined in [2]
% Output:   - x: knapsack solution

function x = knapsack(a,b,c,d)
% Variables
aux = a-d.*b;
aux = [aux;a];
y = sort(aux);
n = length(a);
% 0. Initialization
if c<0 || c>sum(b)
    x = NaN;
    return;
else
    l = 1;
    r = 2*n;
    R = 0;
    L = sum(b);
end
% 1. Test for bracketing
while true
    if r-l == 1
        % 5. Interpolate
        lambda = y(l) + (y(r)-y(l))*(c-L)/(R-L);
        break;
    else
        m = floor((l+r)/2);
    end
    % 3. Compute new value
    aux = (a-y(m))./d;
    for i = 1:n
       aux(i) = max(min(aux(i),b(i)),0);
    end
    C = sum(aux);
    % 4. Update
    if C == c
        lambda = y(m);
        break;
    elseif C > c
        l = m;
        L = C;
    else
        r = m;
        R = C;
    end    
end
x = zeros(n,1);
for i = 1:n
    x(i) = max(min((a(i)-lambda)/d(i),b(i)),0);
end
end

%% References
% [1] Helgason, R., Kennington, J., Lall, H., 1980. A polynomially bounded 
% algorithm for a singly constrained quadratic program. Math. Program. 
% 18 (1), 338-343.

% [2] Pedroso, L. and Batista, P., 2021. Decentralized store-and-forward 
% based strategies for the signal control problem in large-scale congested 
% urban road networks. Transportation Research Part C: Emerging 
% Technologies, 132, p.103412. doi:10.1016/j.trc.2021.103412.


