classdef DTSolverWithInputs <handle
% DTSOLVERWITHINPUTS Direct transcription solver for systems with  open 
% input channel
%
% The optimization can be formulated for multiple simulation runs. 
    properties
        system      %An ODE system
        objective   %Control objective
        initCondition
        
        % Design constraint is a constraint over design variables
        % evaluated once
        % 
        designConstraint 
        % State constraint is a constraint over design and state variables
        % evaluated at each time point, it must be defined for each t
        % vector indivdually
        stateConstraint 
        options  % optimization options
    end
    
    properties
        t         % Cell array
        input     % cell array of function handles
        weight    % cell array of weights
        
        pinit     % Initial guess of parameter
        
        x0        % initial states
        n_control % number of control variables
        
        lb        % lower bound
        ub        % upper bound
        
        n_design  % number of parameters
        n_state   % number of states
    end
    
    methods
        function [obj, Gradient] = f_obj(obj, x)
            % Objective and gradient 
            [obj, Gradient] = f_obj_impl(obj,x);
        end
        function [G, H, JG, JH] = f_con(obj, x)
            % Defect constraints
            [H, JH] = equalityConstraints(obj, x);
            % Plant and state inequality constraints
            [G, JG] = inequalityConstraints(obj, x);
        end
        function [out] = f_solve(obj)
            % Solve the problem
            out = f_solve_impl(obj);
        end
        function h = f_hessian(obj, x, lambda)
            % Hessian
            h = f_hessian_impl(obj, x, lambda);
        end
    end
end

function [out] = f_solve_impl(obj)
% Implemenation of the solver algorithm
    checkSolver(obj);
    
    obj.n_design = length(obj.pinit);
    xinit = obj.initCondition.fun(obj.pinit);
    obj.n_state = length(xinit);
    t = obj.t;
    % Initial control is zero
    if isempty(obj.x0)
        if iscell(obj.t)
            xu = [];
            for i=1:length(obj.t)
                t = obj.t{i};
                input = obj.input{i};
                xu = [xu; initial_sim(obj, t, input)]; %#ok<AGROW> as iteration is small
            end
        else
            xu = initial_sim(obj, t);   
        end
        x0 = [obj.pinit; xu];
    else
        x0 = obj.x0;
    end
    
    options = obj.options;
    if ~isempty(obj.objective.hessian) && ~isempty(obj.designConstraint.hessian)
	% Determine if Hessian is available in the user implemenation
       options = optimset(options, 'Hessian', 'on', ...
           'HessFcn', @(x, lambda) obj.f_hessian(x, lambda));
    end
    
    % Set up to log the intermediate result every 100 steps
    options = optimset(options, 'OutputFcn', @(x_,o_,s_)...
        dtOutput(obj, 100, x_, o_, s_));
    
    [xopt,fopt,flag,output]=fmincon(@(x)obj.f_obj(x),x0,[],[],[],[], ...
        obj.lb,obj.ub,@(x)obj.f_con(x),options);
    out.xopt = xopt;
    out.param = xopt(1:obj.n_design);
    offset = 0;
    for i=1: length(obj.t)
        nt = length(obj.t{i});
        total_length = (obj.n_state+obj.n_control )* nt;
        out.xu{i} = reshape(xopt(obj.n_design + offset +1: obj.n_design+offset+total_length), ...
            (obj.n_state+obj.n_control), nt);
        offset = offset + total_length;
    end
    out.fopt = fopt;
    out.flag = flag;
    out.output = output;
    
end

function xu = initial_sim(obj,t, input)
% Simulate the system for its response using the given parameters and control input
xinit = obj.initCondition.fun(obj.pinit);
[~,x_state] = ode23(...
    @(t,x)obj.system.deriv(t,x,zeros(obj.n_control,1),obj.pinit, input),...
    t, xinit);
nt = length(t);
u0 = zeros(obj.n_control, length(t));
xu = reshape([x_state'*3; u0], (obj.n_state + obj.n_control) * nt, 1);
end


function H = f_hessian_impl(obj, x, lambda)
% Implementation of the Hessian function for Lagrangian
x_state = x(obj.n_design+1:end);
x_d = x(1:obj.n_design);
G = obj.designConstraint.fun(x_d);
m = size(G,1);
F = obj.designConstraint.hessian(x_d, lambda.ineqnonlin(1:m));

state_Hess = sparse(length(x_state), length(x_state));
offset = 0;
for i=1:length(obj.t)
    total_state = length(obj.t{i}) * (obj.n_state + obj.n_control);
    nt = length(obj.t{i});
    w = obj.weight{i};
        
    x_state_frames = reshape(x_state(offset+1:offset+total_state), ...
        obj.n_state + obj.n_control, nt);
    state_Hess(offset+1:offset+total_state, offset+1:offset+total_state) = ...
        w*obj.objective.hessian(x_d, x_state_frames, obj.t{i}); %#ok<SPRIX>
    offset = offset + total_state;
end
H =  [F, sparse(obj.n_design,length(x_state));
    sparse(length(x_state), obj.n_design),  ...
    state_Hess];
end

function [total_f, Gradient] = f_obj_impl(obj ,x)
% Implementation of the objective function 
    x_d = x(1:obj.n_design);
    offset = 0;
    total_control_Gradient = [];
    total_f = 0;
    total_gradient_d = zeros(obj.n_design,1);
    for i=1:length(obj.t)
        t = obj.t{i};
        nt = length(t);
        w = obj.weight{i};
        input = obj.input{i};
                
        total_length = (obj.n_state+obj.n_control)*nt;
        x_state = x(obj.n_design+ offset +1: offset + obj.n_design+total_length);
        offset = offset + total_length;
        x_state_frames = reshape(x_state, obj.n_state + obj.n_control, nt);
        obj_control = w*obj.objective.fun(x_d, x_state_frames, t, input);
        obj_control_Gradient = w*obj.objective.gradient(x_d, x_state_frames, t, ...
            obj.input{i});
        obj_f = obj_control;
        total_control_Gradient = [total_control_Gradient; ...
            obj_control_Gradient]; %#ok<AGROW> as the iteration is small
        total_f = total_f + obj_f;
    
        if isempty(obj.objective.gradient_d)
            gradient_d = zeros(obj.n_design,1);
        else
            gradient_d = w*obj.objective.gradient_d(x_d, x_state_frames, t, ...
                obj.input{i});
        end
        total_gradient_d = total_gradient_d + gradient_d;
    end
    Gradient = [total_gradient_d; total_control_Gradient];
end

function [H, JH] = equalityConstraints(obj, x)
% Implementation of the equality constraints, i.e., the Defect equations 
% for the discretized time steps
    x_d = x(1:obj.n_design); %#ok<NASGU>
    NT= calculateTotalSize(obj);
    M = NT + length(obj.t); % 
    
    H_state = sparse(obj.n_state, M);
    JH_state = sparse(obj.n_state*M, ...
            (obj.n_state + obj.n_control)*NT);
    JH_param = sparse(obj.n_state*M, obj.n_design);
    offset = 0;
    t_1_offset = 0; % increment of (t+1)
    n_offset = 0; 
    
    dimensions = struct('n_state', obj.n_state, 'n_control', obj.n_control);
    
    for i=1:length(obj.t)
        t = obj.t{i};
        nt = length(t);
        total_length = nt * (obj.n_state + obj.n_control);
        
        x_state = x(obj.n_design+offset+1:obj.n_design+offset + total_length);
        x_state_frames = reshape(x_state, obj.n_state + obj.n_control, nt);
        
        xd = x(1:obj.n_design);
        [H, JH] = defectEquation(obj.system, obj.initCondition, obj.input{i}, ...
            obj.t{i}, x_state_frames, xd, dimensions);
        J = JH'; 
        % Fill in the data
        
        m_offset = t_1_offset * obj.n_state;
        m_end = (t_1_offset + nt +1) * obj.n_state;
        H_state(m_offset+1 : m_end) = H; %#ok<SPRIX>
        
        n_end = n_offset + nt*(obj.n_state + obj.n_control);
        
        JH_param(m_offset +1 : m_end , :) = J(:, 1:obj.n_design); %#ok<SPRIX>
        JH_state(m_offset+1 : m_end, ...
            n_offset+1: n_end) = J(:, obj.n_design+1: end); %#ok<SPRIX>
        
        % Move the offset to next simulation
        offset = offset + total_length;
        t_1_offset = t_1_offset + (nt + 1);
        n_offset = n_end;
    end
    
    H = reshape(H_state, numel(H_state),1);
    JH = [JH_param, JH_state]';
end

function [G, JG] = inequalityConstraints(obj,x)
% Implementation of the inequality constraints, including design constraints
% and path constraint (state constraints).

xd = x(1:obj.n_design);
% Add design constraints
if ~isempty(obj.designConstraint)
    G_d = obj.designConstraint.fun(xd);
    JG_d = [obj.designConstraint.jacobian(xd), ...
        zeros(numel(G_d), length(x)-obj.n_design)];
else
    G_d = [];
    JG_d = [];
end

% State constraints
[G_s, JG_s] = computeStateConstraints(obj, x, obj.n_design, xd);
G = [G_d; G_s];
JG = [JG_d; JG_s]';
end

function nt = calculateTotalSize(obj)
% calculate total size
nt = sum(cellfun(@length, obj.t));
end

