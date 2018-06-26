function [system, objective, init, designC, stateC] = simple_sys()
% A very simple system for testing purposes

% Consider a moving object in a 2-D space with a constant speed (design
% variable) and varying angle (control input). Initially the object is at
% [0,0] and the goal is to travel to [1, x]
% 
%  dx/dt = v * cos(u)
%  dy/dt = v * sin(u)
%

system = System;
system.parameters = [];
system.deriv = @(t_,x_,u_, xd_, input_) f( u_, xd_);
system.jacobian= @(t_,x_,u_, xd_, input_) jacobian(u_, xd_);
system.jacobian_d = @(t_,x_,u_, xd_, input_) jacobian_d(u_, xd_);

objective = Objective;
objective.fun = @(xd_, x_, t_, input_) regulate_input(x_, t_, input_);
objective.gradient = @(xd_, xu_, t_, input_) regulate_gradient(xu_, t_, input_);
objective.gradient_d = @(xd_, xin_, t_, input_) 0;

init = InitCondition;
init.fun = @(xd)[0;0];
init.jacobian = @(xd) [0;0];

designC = DesignConstraint;
designC.fun = @designCon;
designC.jacobian = @designConJacobian;

stateC = StateConstraint;
stateC.fun = @stateCon;
stateC.jacobian = @stateConJacobian;
end

function dx = f(u, xd)
dx = [xd * cos(u); xd * sin(u)];
end

function dfdxu = jacobian(u, xd)
% The Jacobian is augmented Jacobian for [x,u] vector
dfdxu = [ 0 0 -xd*sin(u); 0 0 xd*cos(u) ];
end

function dfdxd = jacobian_d(u, ~)
dfdxd = [ cos(u); sin(u)];
end

function o = regulate_input(xu, t, input)
inputseqCell = arrayfun(input, t, 'UniformOutput', false);
inputseq = [inputseqCell{:}]; % Convert cell to mat
distance = xu(1:2,:) - inputseq;
R = diag(t(2:end) - t(1:end-1));
%fwdInt = distance(:,1:end-1);
bwdInt = distance(:,2:end);
weighted = bwdInt'* bwdInt * R;
o = sum(diag(weighted));
end


function J = regulate_gradient(xin, t, input)
% For now, use perturbation
    function o = locRegulate(xu, t, input)
        xu_frames = reshape(xu, 3,numel(xu)/3);
        o = regulate_input(xu_frames, t, input);
    end
xu = reshape(xin, numel(xin),1);
g = perturb_x(@(xu_) locRegulate(xu_, t, input), xu);
J = g';
end

function g = designCon(xd)
% 0.5 <= xd <= 1
g = [xd - 2; 0.5 - xd];
end

function dg = designConJacobian(~)
dg = [1; -1];
end

function g = stateCon(xd, x)
% -1 <= y <= 1
y = x(2);
g = [y - 1; -1 - y];
end

function [dg_d, dg] = stateConJacobian(xd, x)
%  The Jacobian is augmented Jacobian for [x,u] vector
dg = [ 0 1 0; 0 -1 0];
dg_d = [0;0];
end