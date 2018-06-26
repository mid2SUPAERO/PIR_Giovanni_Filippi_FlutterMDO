
classdef SequentialSolverWithInputs < handle
    % SEQUENTIALSOLVER A Sequential solver for MDO autonomous systems
    properties
        system
        objective
        initCondition
        
        designConstraint
        stateConstraint
        
        options   % An array for two steps
    end
    
    properties
        t         %Cell array
        
        input     %Cell array of function handles
        weight    %Cell array of weights
        pinit
        
       
        x0        %initial states
        n_control % number of control variables
        
        lb        %lower bound
        ub        %upper bound
        
        llb       %lower bound for control force
        uub       %upper bound for control force
        
        n_design  % number of parameters
        n_state   % number of states
    end
    
    methods
        function out = f_solve(obj)
            % Sequential solver
            out1 = obj.f_solve_xd();
            xd = out1.xd;
            out2 = obj.f_solve_xu(xd);
            out.xd = out1.xd;
            out.xu_frames = out2.xu;
            out.fopt = out2.fopt; 
            out.step1 = out1;
            out.step2 = out2;
        end
    end
    %% Prototypes
    % These are function prototypes. They act like the declaration in C/C++.
    methods
        function [obj] = f_obj_design(obj, xd, u, xinit)
            % Objective for the plant design problem
            obj = f_obj_design_impl(obj, xd, u, xinit);  
        end
        function [G, H] = f_con_design(obj, xd)
            % Objective for the plant design problem
            [G, H] = f_con_design_impl(obj, xd);
        end
        function out = f_solve_xd(obj)
            % Solve the plant design problem
            out = f_solve_xd_impl(obj);
        end
        function [obj, Gradient] = f_obj_control(obj, xd, xu)
            % Objective for the control design problem
           [obj, Gradient] = f_obj_control_impl(obj,xd, xu);
        end
        
        function [G, H, JG, JH] = f_con_control(obj, xd, x)
            % Constraints for the control design problem
            [H, JH] = equalityControlConstraints(obj, xd, x);
            [G, JG] = inequalityControlConstraints(obj, xd, x);
        end
        function out = f_solve_xu(obj, xd)
            % Solve the control design problem
            out = f_solve_xu_impl(obj, xd);
        end
        
    end
    
end


function yesno = myCheckSolver(obj)
if strcmp(optimget(obj.options(1), 'GradObj'), 'on') && ...
    isempty(obj.objective.gradient_d)
    error('SequentialSolver:NoDesignGradient', ...
        'Design gradient not provided in the user code');
end

if strcmp(optimget(obj.options(1), 'GradObj'), 'on')
    obj.options(1) = optimset(obj.options(1), 'GradObj', 'off');
end
if strcmp(optimget(obj.options(1), 'GradConstr'), 'on')
    obj.options(1) = optimset(obj.options(1), 'GradConstr', 'off');
end

yesno = true;
end

function out = f_solve_xd_impl(obj)
% Solve optimization for design variable xd
checkSolver(obj);
myCheckSolver(obj); % Check the first option

obj.n_design = length(obj.pinit);
xinit = obj.initCondition.fun(obj.pinit);
obj.n_state = length(xinit);

options = obj.options(1);

u = zeros(obj.n_control,1);  % Assume no control input
xd0 = obj.pinit; %initial guess

[xdopt,fopt,flag,output]=fmincon(@(xd)obj.f_obj_design(xd, u, xinit), ...
    xd0,[],[],[],[], ...
    obj.lb,obj.ub,@(xd)obj.f_con_design(xd),options);

out.xd = xdopt;
out.fopt = fopt;
out.flag = flag;
out.output = output;
end

function [total_f] = f_obj_design_impl(obj, xd, u, xinit)
obj.n_state=length(xinit);
total_f = 0;
for i=1:length(obj.t)
    t = obj.t{i};
    xu_frames = simulateSystem(obj, xd, t, u, obj.input{i}, xinit);
    obj_f = obj.weight{i}*obj.objective.fun(xd, xu_frames, t, obj.input{i});
    total_f = total_f + obj_f;
end  
end

function out = f_solve_xu_impl(obj, xd)
checkSolver(obj);
myCheckSolver(obj); % Check the second option
% Solve optimization for design variable xd
obj.n_design = length(obj.pinit);
xinit = obj.initCondition.fun(xd);
obj.n_state = length(xinit);
sim0 = {};

for i=1:length(obj.t)
    sim0{i} = initial_sim(obj, obj.t{i}, xd, obj.input{i});
end

xu0 = vertcat(sim0{:});
options = obj.options(2);
[xuopt,fopt,flag,output]=fmincon(@(xu)obj.f_obj_control(xd, xu), ...
    xu0,[],[],[],[],obj.llb,obj.uub,@(x)obj.f_con_control(xd, x),options);
offset = 0;

for i = 1:length(obj.t)
    nt = length(obj.t{i});
    total_length = (obj.n_state+obj.n_control)*nt;
    out.xu{i} = reshape(xuopt(offset+1: offset+total_length),...
        (obj.n_state+obj.n_control),nt);
    offset = offset + total_length;
end
out.fopt = fopt;
out.flag = flag;
out.output = output;
end

function xu0 = initial_sim(obj, t, xd, input)
% An initial point for control problem
xinit = obj.initCondition.fun(obj.pinit);
obj.n_state = length(xinit);
[~,x_state] = ode23(...
    @(t,x)obj.system.deriv(t,x,zeros(obj.n_control,1), xd, input),...
    t, xinit);
nt = length(t);
xu0 = reshape([x_state'*3; zeros(obj.n_control, nt)], (obj.n_state+obj.n_control)*nt,1);
end

function [total_f, Gradient] = f_obj_control_impl(obj, xd, xu)
% Direct-transcription evaluation of control objective
offset = 0;
total_control_Gradient = [];
total_f = 0;
for i=1:length(obj.t)
    t = obj.t{i};
    nt = length(t);
    w = obj.weight{i};
    total_length = (obj.n_state+obj.n_control) * nt;
    x_state = xu(offset+1: offset+total_length);
    offset = offset + total_length;
    
    x_state_frames = reshape(x_state, obj.n_state + obj.n_control, nt);
    
    obj_control = w *obj.objective.fun(xd,x_state_frames,t, obj.input{i}); 
    obj_control_Gradient = w * obj.objective.gradient(xd, x_state_frames, t,...
        obj.input{i});
    obj_f = obj_control;
    total_control_Gradient = [total_control_Gradient; obj_control_Gradient];
    total_f = total_f + obj_f;   
    Gradient = total_control_Gradient;
end
end

function [H, JH] = equalityControlConstraints(obj, xd, x)
% Direct-transcription evaluation of constraints (defect constraints)
% calculate total size
nt = cellfun(@length, obj.t);
N = sum(nt);
M = sum(nt +1 );

H_state = sparse(obj.n_state * M, 1);
JH_state = sparse(obj.n_state*M,...
    (obj.n_state + obj.n_control)*N); % pre-allocate memory
offset = 0;
t_1_offset = 0; % increment of (t+1)
n_offset = 0;

dimensions = struct('n_state', obj.n_state, 'n_control', obj.n_control);
for j = 1: length(obj.t)
    t = obj.t{j};
    nt = length(t);
    total_length = nt * (obj.n_state+ obj.n_control);
    x_state = x(offset+1:offset+total_length);
    x_state_frames = reshape(x_state, obj.n_state + obj.n_control, nt);
    [H, JH] = defectEquation(obj.system, obj.initCondition, obj.input{j}, ...
        obj.t{j}, x_state_frames, xd, dimensions);
    J = JH'; 
    % Fill in the data
    m_offset = t_1_offset * obj.n_state;
    m_end = (t_1_offset + nt +1) * obj.n_state;
    H_state(m_offset+1 : m_end) = H;
    n_end = n_offset + nt*(obj.n_state + obj.n_control);
    
    JH_state(m_offset+1 : m_end, ...
        n_offset+1: n_end) = J(:, obj.n_design+1: end);
    % Move the offset to next simulation
    offset = offset + total_length;
    t_1_offset = t_1_offset + (nt + 1);
    n_offset = n_end;
end
H = reshape(H_state, numel(H_state), 1);
JH = JH_state'; 
end

function [G_s, JG] = inequalityControlConstraints(obj, xd, x)
% State inequalities
[G_s, JG_s] = computeStateConstraints(obj, x, 0, xd);
JG = JG_s';
end

function [G, H] = f_con_design_impl(obj,xd)
% Design has nonlinear inequality constraints
H = []; 
JH = [];
if ~isempty(obj.designConstraint)
    G = obj. designConstraint.fun(xd);
    JG = obj. designConstraint.jacobian(xd);
else
    G = [];
    JG = [];
end
x = [];
xinit = obj.initCondition.fun(xd);
u = 0; 
for i=1:length(obj.t)
    xu_frames = simulateSystem(obj, xd, obj.t{i}, u, obj.input{i}, xinit);
    x = [x; reshape(xu_frames, numel(xu_frames), 1)];
end
G_s = computeStateConstraints(obj, x, 0, xd);
G = [G; G_s];
end

function xu_frames = simulateSystem(obj, xd, t, u, in, xinit)
% Simulate the system for a constant input u
nt = length(t);
[~,x_state_sim] = ode23(@(t_, x_)obj.system.deriv(t_, x_, u, xd, in) , ...
    t, xinit);
xu_frames = [x_state_sim'; repmat(u, 1 ,nt)];
end


    
    
    
    

