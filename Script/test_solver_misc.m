function test_suite = test_solver_misc()
% Test various stuff in the solver library
initTestSuite;
end

function testPerturbX()
J  = perturb_x(@(x)[1 1]*x, [1;2], 1e-4);
assertElementsAlmostEqual(J, [1, 1], 'relative', 1e-3);
end

function testPerturbX2()
J  = perturb_x(@(x)[1 1]*x, [1;2]);
assertElementsAlmostEqual(J, [1, 1], 'relative', 1e-3);
end

function testHessianX()
J = hessian_x(@(x) x'* 3 * x, [1;2], 1e-5);
assertEqual(size(J), [1,1]);
assertElementsAlmostEqual(J{1}, [6 0; 0 6], 'relative', 1e-3);
end


function testStateConstraint()
con = StateConstraint;

con.fun = @(xd, x)(x-0.5*xd);

con.jacobian = @jac;

G = zeros(2,1);
JG = zeros(2,3);
state_frames = [1,1];
nc = 1;

[G, JG] = fillInStateConstraint(G, JG, 0, ...
    1, [0 1], 1, state_frames, con, nc, false);
assertElementsAlmostEqual(G, [0.5;0.5], 'relative', 1e-3);
assertElementsAlmostEqual(JG, [-0.5 1 0; -0.5 0 1], 'relative', 1e-3);

end

function testStateConstraintNoDesign()
con = StateConstraint;

con.fun = @(xd, x)(x-0.5*xd);

con.jacobian = @jac;

G = zeros(2,1);
JG = zeros(2,3);
state_frames = [1,1];
nc = 1;

[G, JG] = fillInStateConstraint(G, JG, 0, ...
    1, [0 1], 1, state_frames, con, nc, true);
assertElementsAlmostEqual(G, [0.5;0.5], 'relative', 1e-3);
assertElementsAlmostEqual(JG, [0 1 0; 0 0 1], 'relative', 1e-3);

end


function [jd, js] = jac(xd,x)
jd = -0.5;
js = 1;
end