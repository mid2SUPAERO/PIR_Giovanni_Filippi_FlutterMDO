function test_suite = test_simple_sys()
% Test various stuff in the solver library
initTestSuite;
end

function testSim()
% Test that simple system can simulate
[system, ~, init] = simple_sys();
opt = odeset('RelTol', 1e-2, 'AbsTol', 1e-3);

[t,x] = ode23(@(t_,x_) system.deriv(t_,x_, 0, 1, @(t)[t;0]), [0 1], ...
    init.fun(1), opt);
assertElementsAlmostEqual(t - x(:,1), zeros(size(t)), 'relative', 1e-3);
assertElementsAlmostEqual(x(:,2), zeros(size(t)), 'relative', 1e-3);
end

function testDTSolverWithInput()
% Test that the tracking of simple system for [t; 0] can be solved
[system, objective, init, designC, stateC] = simple_sys();

s = DTSolverWithInputs();
s.system = system;
s.objective = objective;
s.initCondition = init;
s.designConstraint = designC;
s.stateConstraint = stateC;

t = 0:0.1:1;
s.n_control = 1;
s.t ={t};
s.input = {@(t)[t;0]};
s.pinit = 0.6;
s.x0 = [];
s.lb = [];
s.ub = [];
s.weight = {1};

s.options = OptionFactory.getInstance.makeOption('');
out = s.f_solve();

assertElementsAlmostEqual(t - out.xu{1}(1,:), zeros(size(t)), 'absolute', 1e-2);
assertElementsAlmostEqual(out.xu{1}(2,:), zeros(size(t)), 'absolute', 1e-2);
assertElementsAlmostEqual(out.fopt, 0, 'absolute', 1e-3);

end

function testDTSolverWithControlConstraints()
% Test that the tracking of simple system for [t; 0] can be solved
[system, objective, init, designC, stateC] = simple_sys();

s = DTSolverWithInputs();
s.system = system;
s.objective = objective;
s.initCondition = init;
s.designConstraint = designC;
s.stateConstraint = stateC;

t = 0:0.1:1;
s.n_control = 1;
s.t ={t};
s.input = {@(t)[t;0]};
s.pinit = 0.6;
s.x0 = [];
[lb,ub] = calculateControlBounds(length(t), 2, 1, -1, 0.1);
s.lb = [-inf; lb];
s.ub = [inf; ub];
s.weight = {1};

s.options = OptionFactory.getInstance.makeOption('');
out = s.f_solve();
while(out.flag ~=1)
    s.x0 = out.xopt;
    out = s.f_solve();
end

assertElementsAlmostEqual(t - out.xu{1}(1,:), zeros(size(t)), 'absolute', 1e-2);
assertElementsAlmostEqual(out.xu{1}(2,:), zeros(size(t)), 'absolute', 1e-2);
assertElementsAlmostEqual(out.fopt, 0, 'absolute', 1e-4);
end

function testSequentialSolverWithInput()
% Test that the tracking of simple system for [t; 0] can be solved
[system, objective, init, designC, stateC] = simple_sys();

s = SequentialSolverWithInputs();
s.system = system;
s.objective = objective;
s.initCondition = init;
s.designConstraint = designC;
s.stateConstraint = stateC;

t = 0:0.1:1;
s.n_control = 1;
s.t ={t};
s.input = {@(t)[t;0]};
s.pinit = 0.6;
s.x0 = [];
s.lb = [];
s.ub = [];
s.weight = {1};

s.options = OptionFactory.getInstance.makeOption('');
s.options(2) = OptionFactory.getInstance.makeOption('');
out = s.f_solve();

end

function testDTSolverWithInput_rampInput()
% Test that the tracking of simple system for [t;t/2] can be solved
[system, objective, init, designC, stateC] = simple_sys();

s = DTSolverWithInputs();
s.system = system;
s.objective = objective;
s.initCondition = init;
s.designConstraint = designC;
s.stateConstraint = stateC;

t = 0:0.1:1;
s.n_control = 1;
s.t ={t};
s.input = {@(t)[t; t/2]};
s.pinit = 0.6;
s.x0 = [];
s.lb = [];
s.ub = [];
s.weight = {1};

s.options = OptionFactory.getInstance.makeOption('');
s.options = optimset(s.options, 'TolFun', 1e-6);
        
out = s.f_solve();

assertElementsAlmostEqual(out.xu{1}(1,:), t, 'absolute', 5e-2);
assertElementsAlmostEqual(out.xu{1}(2,:), t/2, 'absolute', 5e-2);

assertElementsAlmostEqual(out.fopt, 0, 'absolute', 1e-4);
assertElementsAlmostEqual(out.param, sqrt(1.25), 'absolute', 5e-2);


end
