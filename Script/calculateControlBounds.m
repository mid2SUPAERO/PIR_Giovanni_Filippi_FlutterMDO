function [lb,ub] = calculateControlBounds(nt, nx, nu, ulb, uub)
% CalculateControlBounds compute two vectors for control upper and lower
% bounds
n = nt * (nx + nu);
lb = -inf(n,1);
ub = inf(n,1);

for i=1:nt
    lb((i-1)*(nx+nu)+nx+1:(i-1)*(nx+nu)+nx+nu) = ulb;
    ub((i-1)*(nx+nu)+nx+1:(i-1)*(nx+nu)+nx+nu) = uub;
end

end
