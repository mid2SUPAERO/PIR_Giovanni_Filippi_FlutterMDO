% 
% Author: Giovanni Filippi
% Date: 29/06/18
%%
% Prepare the input data
Sgust = gust_cos_input(15,0.6);
Sturb = gust_turb_input(15,0.6);

% open loop
xd = zeros(7,1);
xd(1) = 7.5;            % s: semi span (m)
xd(2) = 2;              % c: chord (m)
xd(3) = 100;            % m: unit mass / area of wing (Kg/m2)
xd(4) = 5;              % kappa_freq: flapping freq (Hz)
xd(5) = 10;             % theta_freq: pitch freq (Hz)
xd(6) = 0.5;            % perc_xcm: percentual of position of centre of mass from nose (m)
xd(7) = 0.48;           % perc_xf: percentual of position of flexural axis from nose (m)

[system, objective, init, designCon, stateCon] = wing_new_objective_open();

% Step size for gust
opt = odeset('RelTol', 1e-2, 'AbsTol', 1e-3);
[t01,~] = ode23(@(t,x)system.deriv(t,x,0, xd, @(t)lookup_u(Sgust,t)), [0 5], init.fun(xd),opt);

% Step size for turbulence
[t02,~] = ode23(@(t,x)system.deriv(t,x,0, xd, @(t)lookup_u(Sturb,t)), [0 5], init.fun(xd),opt);      %solve the system x'=Ax+Bu+b...

s = DTSolverWithInputs();
s.system = system;
s.objective = objective;
s.initCondition = init;
s.designConstraint = designCon;
s.stateConstraint = stateCon;

s.options = OptionFactory.getInstance.makeOption('Display');
s.options = optimset(s.options, 'TolFun', 1e-6);
s.options = optimset(s.options, 'MaxIter', 500);
s.n_control = 1;
s.t ={t01, t02};
s.input = {@(t)lookup_u(Sgust, t),@(t)lookup_u(Sturb, t)};
s.weight = {1, 1};
s.lb = [4 1 70 3 8 0.1 0.1];
s.ub = [10 3 130 7 12 0.9 0.9];

s.pinit =xd; % initial parameterc
s.x0 = [];
fprintf('\nDirect Transcription\n');
starttime = cputime;
out1 = s.f_solve();
total_time{1} = cputime - starttime;
xp1 = out1.xopt(1:7);
clear xd
%%
f1{1} = objective.fun(xp1, out1.xu{1},t01, @(t)lookup_u(Sgust,t));
f1{2} = objective.fun(xp1, out1.xu{2},t02, @(t)lookup_u(Sturb,t));




