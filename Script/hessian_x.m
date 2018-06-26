function F  = hessian_x(Fun, x0, varargin)
% Finite difference evaluation of the Hessian of a function
%
% e.g.,
%  >> F = hessian(@(x)(x^2), 1)
%


if nargin < 5
    pert = 1e-5;
else
    pert = varargin{1};
end
f0 = Fun(x0);
m = length(f0);
n = length(x0);

% Simple single-sided perturbation
for k=1:m
    F{k} = zeros(n,n);
end

for i=1:n
    if length(pert) >1
        delta_i = pert(i);
    else
        delta_i = pert;
    end
    x = x0;
    x(i) = x(i) + delta_i;
    xi  = x;
    f_i = Fun(xi);   
    for j=1: n
        if length(pert) >1
            delta_j = pert(i);
        else
            delta_j = pert;
        end
        xj = x0;
        xj(j)  = xj(j) + delta_j;
        xij = xi;
        xij(j) = xij(j) + delta_j;
        
        f_i_j = Fun(xij);
        f_j =  Fun(xj);
        
        for k=1:m
            F{k}(i,j) = (f_i_j(k) - f_i(k) - f_j(k) + f0(k)) / delta_i/delta_j;
        end
    end
end
end