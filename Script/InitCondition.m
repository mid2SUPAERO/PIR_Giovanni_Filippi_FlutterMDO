classdef InitCondition
    % Initial Condition of the system
    %
    % The initial condition of a system is a function of a few variables.
    % If number of variables is zero, the initial condition is a constant.
    %

    % TODO 
    % - Test the case where number of variables is not zero
    
        
    properties
        fun 
        jacobian
    end
end
