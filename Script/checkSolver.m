function checkSolver(obj)
% checkSolver checks the solver option for appropriate settings.

% A solver must have setup its options
if isempty(obj.options)
    error('Solver:NoOption', 'Solver has not specified option');
end

end