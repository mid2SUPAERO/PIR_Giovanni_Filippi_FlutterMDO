classdef DesignConstraint
    % Design constraint
    %
    % Design constraints are algebraic constraints on the design variables.
    % A DesignConstraint must provide the constraint function and its 
    % Jacobian as function handles.
    %
    % For example, define a Design Constraint x(1) < 2 as
    %
    % >> o = DesignConstraint
    % >> o.fun = @(x)(x-2)
    % >> o.jacobian = @(x)1
    %
    properties
        % nonlinear functions
        fun
        
        % The Jacobian of the function fun
        jacobian
        
        % The Hessian of the Lagrangian function
        hessian  
        
    end
end