function J  = perturb_x(fun, x0, varargin)
% Finite difference

if nargin < 3
    pert = 1e-3;
else
    pert = varargin{1};
end
f0 = fun(x0);   %??????????
m = length(f0);
n = length(x0);
% Simple single-sided perturbation
J = zeros(m,n);
for i=1: n
    if length(pert) >1
        delta = (abs(x0(i)) + 1e-3) * pert(i);
    else
        delta = (abs(x0(i)) + 1e-3) * pert;
    end
    x = x0;
    x(i) = x(i) + delta;
    J(:,i) = (fun(x) - f0)/delta;
end
end