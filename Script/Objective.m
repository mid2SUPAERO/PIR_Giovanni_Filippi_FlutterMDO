classdef Objective
% An objective function formulated for state trajectories

% The trajectory is a matrix where each column represents a 'frame' of
% states, e.g., [chi(1),chi(2),chi(3)...] where
% each chi is an extended state vector, i.e., chi= [x; u] where
% x is state vector and u is the control vector.

    methods
    end
    
    properties
        fun      % xout = fun(x)
        gradient % 
        hessian
        
        fun_d
        gradient_d
        
        QR % may be empty
        
    end
end
