function myplot_results(platform)
% Generate plots from saved results
% Example myplot_results('pcwin64')

platform = upper(platform);

% Plot results for dt
dt = load(['dt_results_' platform]);
[~,~,~,~,stateCon,objComp] = wing_new_objective_open();
myplot_dt(dt.out1, dt.t01, dt.t02, objComp, stateCon);

% Plot results for seq
seq = load(['seq_results_' platform]);
myplot_seq(seq.out2, seq.t01, seq.t02, objComp, stateCon);

end

function myplot_dt(out, t01, t02, objComp, stateCon)
% Generate plots for dt
ramp_in = gust_cos_input(15,0.6);
zdot = gust_turb_input(15,0.6);


% Plot the k, theta
myplot_signal(t01, ...
    out.xu{1}(1,:)*180/pi,t01,out.xu{1}(2,:)*180/pi);
myplot_ylabel('angles (deg)');
myplot_legend('k','\theta');
set(gca, 'YLim', [-1 1]);

% Plot control input
myplot_signal(t01, ...
    out.xu{1}(5,:)*180/pi);
myplot_ylabel('Control Input (deg)');
set(gca, 'YLim', [-1 3]);
% Plot the three different criteria
wt = objComp(out.param, out.xu{1}, t01, @(t)lookup_u(ramp_in,t));
% Plot control input
myplot_signal(t01(2:end), wt(1,:), 'k--', ...
    t01(2:end), wt(2,:), '-', ...
    t01(2:end), wt(3,:), '-.');
myplot_ylabel('Objective');
set(gca, 'YLim', [0 0.06]); 
legend({'Handling', 'Control', 'Comfort'});
set(gca, 'YLim', [0 10000]);


% Plot the k, theta
myplot_signal(t02, ...
    out.xu{2}(1,:)*180/pi,t02, out.xu{2}(2,:)*180/pi);
myplot_ylabel('angles (deg)');
myplot_legend('k','\theta');
set(gca, 'YLim', [-2 2]);

% Plot control input
myplot_signal(t02, ...
    out.xu{2}(5,:)*180/pi);
myplot_ylabel('Control Input (deg)');
set(gca, 'YLim', [-15 15]);

% Plot the three different criteria
wt = objComp(out.param, out.xu{2}, t02, @(t)lookup_u(zdot,t));
% Plot control input
myplot_signal(t02(2:end), wt(1,:), 'k--', ...
    t02(2:end), wt(2,:), '-', ...
    t02(2:end), wt(3,:), '-.');
myplot_ylabel('Objective');
set(gca, 'YLim', [0 10000]); 
legend({'Handling', 'Control', 'Comfort'});
y1 = evaluateStateConstraints(t01, out.param, out.xu{1}, stateCon(1));
myplot_signal(t01, y1);
myplot_ylabel('State constraints');
y2 = evaluateStateConstraints(t02, out.param, out.xu{2}, stateCon(2));
myplot_signal(t02, y2);
myplot_ylabel('State constraints');

end

function myplot_seq(out, t01, t02, objComp, stateCon)

ramp_in = gust_cos_input(15,0.6);
zdot = gust_turb_input(15,0.6);


% Plot the k, theta
myplot_signal(t01, ...
    out.xu_frames{1}(1,:)*180/pi,t01,out.xu_frames{1}(2,:)*180/pi);
myplot_ylabel('angles (deg)');
myplot_legend('k','\theta');
set(gca, 'YLim', [-1 1]);

% Plot control input
myplot_signal(t01, ...
    out.xu_frames{1}(5,:)*180/pi);
myplot_ylabel('Control Input (deg)');
set(gca, 'YLim', [-1 3]);

% Plot the three different criteria
wt = objComp(out.xd, out.xu_frames{1}, t01, @(t)lookup_u(ramp_in,t));
% Plot control input
myplot_signal(t01(2:end), wt(1,:), 'k--', ...
    t01(2:end), wt(2,:), '-', ...
    t01(2:end), wt(3,:), '-.');
myplot_ylabel('Objective');
legend({'Handling', 'Control', 'Comfort'});
set(gca, 'YLim', [0 10000]);

% Plot the k, theta
myplot_signal(t02, ...
    out.xu_frames{2}(1,:)*180/pi,t02,out.xu_frames{2}(2,:)*180/pi);
myplot_ylabel('angles (deg)');
myplot_legend('k','\theta');
set(gca, 'YLim', [-2 2]);

% Plot control input
myplot_signal(t02, ...
    out.xu_frames{2}(5,:)*180/pi);
myplot_ylabel('Control Input (deg)');
set(gca, 'YLim', [-15 15]);

% Plot the three different criteria
wt = objComp(out.xd, out.xu_frames{2}, t02, @(t)lookup_u(zdot,t));
% Plot control input
myplot_signal(t02(2:end), wt(1,:), 'k--', ...
    t02(2:end), wt(2,:), '-', ...
    t02(2:end), wt(3,:), '-.');
myplot_ylabel('Objective');
set(gca, 'YLim', [0 10000]); 
legend({'Handling', 'Control', 'Comfort'});

y1 = evaluateStateConstraints(t01, out.xd, out.xu_frames{1}, stateCon(1));
myplot_signal(t01, y1);
myplot_ylabel('State constraints');
y2 = evaluateStateConstraints(t02, out.xd, out.xu_frames{2}, stateCon(2));
myplot_signal(t02, y2);
myplot_ylabel('State constraints');
end