function [H, JH] = defectEquation(system, initCondition, input, ...
    t, x_state_frames, x_design, dimensions)
%DEFECTEQUATION computes the discretized defect equation for an ODE system
%
%   [H, JH] = defectEquation(system, initcondition, t, x, x_design, dimensions)
%
%   where H is the defect vector and JH is the Jacobian of the defect 
%   vector. 
%
% See also DTSOLVER, SEQUENTIALSOLVER 

% Requires
%   * system:  a valid ODE System object
%   * initCondition: a function that implements the init condition
%   * t: a time vector
%   * x: a combined state-control frame vectors.
%   * x_design: the design variables. 
%   * dimensions: a struct that specifies the number of state, number of
%     design variables, number of control and number of free input.
%
% Returns
%   * H - n-by-1 vector of defects 
%   * JH - m-by-n Jacobian of the defects 

n_state = dimensions.n_state;
n_control = dimensions.n_control;

n_design = length(x_design);

if isempty(input)
    % No input function is given
    sysDeriv = system.deriv;
    sysJacobian = system.jacobian;
else
    % bind input to the last argument of system deriv
    sysDeriv = @(t_,x_,u_,xd_)system.deriv(t_,x_,u_,xd_, input);
    sysJacobian = @(t_,x_,u_,xd_) system.jacobian(t_,x_,u_,xd_, input);
end

% Calculate h and Jh
params = x_design;
xinit = initCondition.fun(params);
H0 = xinit - x_state_frames(1:n_state,1);

nt = length(t);
H_state = zeros(n_state, nt);
JH_state = sparse(n_state*nt, ...
    (n_state + n_control)*nt);
for i=2:nt
    state_i = x_state_frames(1:n_state,i);
    control_i = x_state_frames(n_state+1:n_state+n_control,i);
    
    xd_i = sysDeriv(t(i), state_i, control_i, params);  %dx(i)
    state_i_1 = x_state_frames(1:n_state,i-1);
    control_i_1 = x_state_frames(n_state+1:n_state+n_control,i-1);
    
    xd_i_1 = sysDeriv(t(i-1) ,state_i_1, control_i_1, params);  %dx(i-1)
    dti = t(i) - t(i-1);
    H_state(:,i) = state_i - state_i_1 ...
        - dti/2 * (xd_i + xd_i_1);  %metodo dei trapezi (pag 3)
    
    dfdx_i = sysJacobian(t(i), state_i, control_i, params);
    dfdx_i_1 = sysJacobian(t(i-1), state_i_1, control_i_1,params);
    JH_state((i-1)*n_state+1:i*n_state, ...
        (i-1)*(n_state+n_control)+1:i*(n_state +n_control)) ...
        = [speye(n_state), zeros(n_state, n_control)] - dti/2*dfdx_i;
    JH_state((i-1)*n_state+1:i*n_state, ...
        (i-2)*(n_state + n_control) +1:(i-1)*(n_state + n_control)) ...
        = [-speye(n_state), zeros(n_state, n_control)] - dti/2*dfdx_i_1;
end

JH_param = sparse(n_state*nt, n_design);
for i=2:nt
    state_i = x_state_frames(1:n_state,i);
    control_i = x_state_frames(n_state+1:n_state+n_control,i);
    state_i_1 = x_state_frames(1:n_state,i-1);
    control_i_1 = x_state_frames(n_state+1:n_state+n_control,i-1);
    dti = t(i) - t(i-1);
    if ~isempty(system.jacobian_d)
        % Has non-empty design variable Jacobian, i.e.,
        % Derivative explicitly depends on the design variable
        dfdpi = system.jacobian_d(t(i), state_i, control_i, params);
        dfdpi_1 = system.jacobian_d(t(i-1), state_i_1, control_i_1,params);
        JH_param((i-1)*n_state +1: i*n_state, :) = -dti/2* (dfdpi + dfdpi_1);
    else
        % Derivative does not depend on design variables
        % These entries are left to be zeros.
    end
end

JH = [JH_param, JH_state; ...
    initCondition.jacobian(params), ...
    -speye(n_state), zeros(n_state,n_control), ...
    zeros(n_state, ...
    (nt-1)*(n_state+n_control))]';
H = [reshape(H_state, numel(H_state), 1); H0];

end
