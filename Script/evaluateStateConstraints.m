function y = evaluateStateConstraints(t, xd, xu, stateCon)
% evaluateStateConstraints computes inequality state constraints for the given
% state trajectory. 

% Note this function is not used in the DTSolver, it is used for generating plots.

y1 = stateCon.fun(xd, xu);
m = length(y1);  % determine the dimension of constraint
y = zeros(m, length(t));
for i=1:length(t)
    y_i = stateCon.fun(xd, xu(:,i));
    y(:,i) = reshape(y_i, m,1);
end
end