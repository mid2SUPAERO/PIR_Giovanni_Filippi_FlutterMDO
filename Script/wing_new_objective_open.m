function [system, objective, init, designC, stateC, objComp] = ...
    wing_new_objective_open()
% Quarter-car new objective with design and state constraints

% Design variables
s = 7.5;            % s: semi span (m)
c = 2;              % c: chord (m)
m = 100;            % m: unit mass / area of wing (Kg/m2)
kappa_freq = 5;     % kappa_freq: flapping freq (Hz)
theta_freq = 10;    % theta_freq: pitch freq (Hz)
perc_xcm = 0.5;   % perc_xcm: position of centre of mass from nose (m)
perc_xf = 0.48;   % perc_xf: position of flexural axis from nose (m)

xd = [s; c; m; kappa_freq; theta_freq; perc_xcm; perc_xf];

% Parameters
param.Mthetadot = -1.2;   % unsteady aero damping term
param.a1 = 2*pi;          % lift curve slope

param.V = 100;            % AIRSPEED
param.rho=1.225;          %air density

system = System;
system.parameters = param;
system.deriv  = @(t,x,u,xd, input) f(t,x,u,xd, param, input);
system.jacobian = @(t,x,u,xd, input) jacobian(t,xd, param);
system.jacobian_d = @(t,x,u,xd) jacobian_d(t,x, xd, param);

objective = Objective;
objective.fun = @(x_d,xin, t,input) obj_f(xin, x_d, t, param, input);
objective.gradient = @(x_d,xin,t, input) obj_f_gradient(xin, x_d, t, param);
objective.hessian =  @(x_d,xin,t) obj_f_hessian(xin, x_d, t, param);
objective.gradient_d = @(x_d,xin,t,input)obj_f_design_gradient(xin,x_d,t,param,input);

objComp = @(x_d,xin, t,input) obj_components(xin, x_d, t, param, input);

init = InitCondition;

init.fun = @(xd_)X0(xd_, param);
init.jacobian = @(xd_) X0Jacobian(xd_, param);

designC = DesignConstraint;
designC.fun = @(xd_) designConstraint(xd_, param);
designC.jacobian = @(xd_) designConJacobian(xd_, param);

stateC = StateConstraint;
stateC(2) = StateConstraint;
stateC(1).fun = @(xd_, x_) rampStateCon(xd_, x_, param);
stateC(1).jacobian = @(xd_, x_) rampConJacobian(xd_, x_, param);
stateC(2).fun = @(xd_, x_) roughStateCon(xd_, x_, param);
stateC(2).jacobian = @(xd_, x_) roughConJacobian(xd_, x_, param);
end

function dx = f(t, x, u, xd, param, input)

% design varables
s = xd(1);          % semi span  
c = xd(2);          % chord
m = xd(3);          % unit mass / area of wing
kappa_freq = xd(4); % flapping freq in Hz
theta_freq = xd(5); % pitch freq in Hz
perc_xcm = xd(6);   % percentual position of centre of mass from nose
perc_xf = xd(7);    % percentual position of flexural axis from nose

xcm = centre_of_mass(xd);
xf = flexural_axis(xd);

% set up system matrices
a11=(m*s^3*c)/3 ;                       % I kappa
a22= m*s*(c^3/3 - c*c*xf + xf*xf*c);    % I theta
a12 = m*s*s/2*(c*c/2 - c*xf);           %I kappa theta
a21 = a12;
A=[a11,a12;a21,a22];                    % inertia matrix

% gust vector
F_gust = param.rho*param.V*c*s*[s/4 c/2]';

% system
J = jacobian(t, xd, param);
As = J(1:4,1:4);
Bs = J(1:4, 5);
h = [0; 0; A\F_gust];
if ~isempty(input)   
    dx= As* x + Bs * u + h * input(t); 
else
    dx = As*x + Bs*u ;
end
end


function J = jacobian(~, xd, param)

% design varables
s = xd(1);          % semi span  
c = xd(2);          % chord
m = xd(3);          % unit mass / area of wing
kappa_freq = xd(4); % flapping freq in Hz
theta_freq = xd(5); % pitch freq in Hz
perc_xcm = xd(6);   % percentual position of centre of mass from nose
perc_xf = xd(7);    % percentual position of flexural axis from nose

xcm = centre_of_mass(xd);
xf = flexural_axis(xd);
e = eccentricity(xd);

% set up system matrices
a11=(m*s^3*c)/3 ;                    % I kappa
a22= m*s*(c^3/3 - c*c*xf + xf*xf*c); % I theta
a12 = m*s*s/2*(c*c/2 - c*xf);        %I kappa theta
a21 = a12;
k1=(kappa_freq*pi*2)^2*a11;          % k kappa
k2=(theta_freq*pi*2)^2*a22;          % k theta
A=[a11,a12;a21,a22];                 % inertia matrix
E=[k1 0; 0 k2];                      % structural stiffness matrix
C=param.rho*param.V*[c*s^3*param.a1/6,0;-c^2*s^2*e*param.a1/4,-c^3*s*param.Mthetadot/8]; % aero damping
K=(param.rho*param.V^2*[0,c*s^2*param.a1/4;0,-c^2*s*e*param.a1/2])+[k1,0;0,k2] ;   % aero and structural stiffness

% control surface vector
EE = .1;  % fraction of chord made up by control surface
ac = param.a1/pi*(acos(1-2*EE) + 2*sqrt(EE*(1-EE)));
bc = -param.a1/pi*(1-EE)*sqrt(EE*(1-EE));
F_control = param.rho*param.V^2*c*s*[-s*ac/4 c*bc/2]';

%jacobian
As = [ 0 0 1 0;
       0 0 0 1;
    -inv(A)*K -inv(A)*C];
Bs = [0; 0; A\F_control];
J = [As, Bs];
end

function e = eccentricity(xd)
% Calculate the eccentricity e between flexural axis and aero centre e from the design variables
c = xd(2);
xf = xd(7)*c;
e = xf/c - 0.25;
end

function xcm = centre_of_mass(xd)
% Calculate the position of centre of mass from nose
xcm = xd(6)*xd(2);
end

function xf = flexural_axis(xd)
% Calculate the position of flexural axis from nose
xf = xd(7)*xd(2);
end

function J = jacobian_d(t, x, xd, param)
J = perturb_x(@(xd)f(t,x,0, xd, param, []), xd);
end

function val = obj_f(xu, x_d, t, param, input)

% design varables
s = x_d(1);                 % semi span  
c = x_d(2);                 % chord
xf = flexural_axis(x_d);    %flexural axis

xu_frame = xu;
val = 0;
R = Weight();
xu_frame_zs = zeros(6,length(t));

for i=1:(size(xu_frame,2)-1)
    dx = f(t(i),xu_frame(1:4,i), xu_frame(5,i), x_d, param, input);
    z_acc = s*dx(3) + xf*dx(4);
    xu_frame_zs(:,i) = [ [s; xf; 1; 1; 1].*xu_frame(:,i); z_acc];
    val = val+ (t(i+1)-t(i))*(xu_frame_zs(:,i)'*R*xu_frame_zs(:,i)+R(1,1)*xu_frame(1)*xu_frame(2)*s*xf/2);
end
end

% For plotting the components of the objective functions
function wt = obj_components(xu, x_d, t, param, input)

% design varables
s = x_d(1);               % semi span  
c = x_d(2);               % chord
xf = flexural_axis(x_d);  %flexural axis

xu_frame = xu;
R = Weight_components();
wt = zeros(length(R), length(t)-1);
xu_frame_zs = zeros(6,length(t));

for i=1:(size(xu_frame,2)-1)
    dx = f(t(i),xu_frame(1:4,i), xu_frame(5,i), x_d, param, input);
    z_acc = s*dx(3) + xf*dx(4);
    xu_frame_zs(:,i) = [ [s; xf; 1; 1; 1].*xu_frame(:,i); z_acc];
    for j = 1:length(R)
        wt(j,i) = xu_frame_zs(:,i)'*R{j}*xu_frame_zs(:,i);
    end
end
end

function J = obj_f_gradient(xu_frame, x_d, t, param)
nt = numel(xu_frame)/5;
R_gradient = Weight_gradient(x_d,param);
J = zeros(numel(xu_frame),1);
for i=1:(nt-1)
    J((i-1)*5+1:i*5) = 2*(t(i+1)-t(i))*R_gradient*(xu_frame(:,i));
end
end

function H = obj_f_hessian(xin, x_d, t, param)
nt =  numel(xin)/5;
R_gradient = Weight_gradient(x_d,param);
H = zeros(numel(xin), numel(xin));
for i=1:nt
    for j=1:5
        for k =1:5
        H((i-1)*5+j,(i-1)*5+k) = R_gradient(j,k);
        end
    end
end

end

function x0 = X0(xd, param)
x0 = [0;0;0;0];
end

function J = X0Jacobian(xd, param)
x0 = X0(xd, param);
J = zeros(size(x0,1), size(xd,1));
end


function R = Weight()
r1 = 1e3;
r2 = 1e-2;
q = 1e2;
R = diag([r1, r1, 0, 0, q, r2]);
end


function R = Weight_components()
r1 = 1e3;
r2 = 1e-2;
q = 1e2;
R{1} = diag([r1, r1, 0, 0, 0,0]);
R{2} = diag([0 0 0 0 q 0]);
R{3} = diag([0 0 0 0 0 r2]);
end

function R_gradient = Weight_gradient(xd,param)

% design varables
s = xd(1);          % semi span  
c = xd(2);          % chord
m = xd(3);          % unit mass / area of wing
kappa_freq = xd(4); % flapping freq in Hz
theta_freq = xd(5); % pitch freq in Hz
perc_xcm = xd(6);   % percentual position of centre of mass from nose
perc_xf = xd(7);    % percentual position of flexural axis from nose

xcm = centre_of_mass(xd);
xf = flexural_axis(xd);

e = eccentricity(xd);

% set up system matrices
a11=(m*s^3*c)/3 ;                    % I kappa
a22= m*s*(c^3/3 - c*c*xf + xf*xf*c); % I theta
a12 = m*s*s/2*(c*c/2 - c*xf);        %I kappa theta
a21 = a12;
k1=(kappa_freq*pi*2)^2*a11;          % k kappa
k2=(theta_freq*pi*2)^2*a22;          % k theta
A=[a11,a12;a21,a22];                 % inertia matrix
E=[k1 0; 0 k2];                      % structural stiffness matrix
C=param.rho*param.V*[c*s^3*param.a1/6,0;-c^2*s^2*e*param.a1/4,-c^3*s*param.Mthetadot/8]; % aero damping
K=(param.rho*param.V^2*[0,c*s^2*param.a1/4;0,-c^2*s*e*param.a1/2])+[k1,0;0,k2] ;   % aero and structural stiffness

% gust vector
F_gust = param.rho*param.V*c*s*[s/4 c/2]';

% control surface vector
EE = .1;  % fraction of chord made up by control surface
ac = param.a1/pi*(acos(1-2*EE) + 2*sqrt(EE*(1-EE)));
bc = -param.a1/pi*(1-EE)*sqrt(EE*(1-EE));
F_control = param.rho*param.V^2*c*s*[-s*ac/4 c*bc/2]';

MK = A\K; MC = A\C; MFC = A\F_control; MFG = A\F_gust;

r1 = 1e3;
r2 = 1e-2;
q = 1e2;

R_gradient = [
    r1*s^2+r2*(MK(1,1)*s+MK(2,1)*xf)^2, r1*xf*s+r2*(MK(1,2)*s+MK(2,2)*xf)*(MK(1,1)*s+MK(2,1)*xf),  r2*(MC(1,1)*s+MC(2,1)*xf)*(MK(1,1)*s+MK(2,1)*xf), r2*(MC(1,2)*s+MC(2,2)*xf)*(MK(1,1)*s+MK(2,1)*xf), -r2*(MFC(1)*s+MFC(2)*xf)*(MK(1,1)*s+MK(2,1)*xf);...
    r1*s*xf+r2*(MK(1,1)*s+MK(2,1)*xf)*(MK(1,2)*s+MK(2,2)*xf), r1*xf^2+r2*(MK(1,2)*s+MK(2,2)*xf)^2, r2*(MC(1,1)*s+MC(2,1)*xf)*(MK(1,2)*s+MK(2,2)*xf), r2*(MC(1,2)*s+MC(2,2)*xf)*(MK(1,2)*s+MK(2,2)*xf), -r2*(MFC(1)*s+MFC(2)*xf)*(MK(1,2)*s+MK(2,2)*xf);...
    r2*(MK(1,1)*s+MK(2,1)*xf)*(MC(1,1)*s+MC(2,1)*xf), r2*(MK(1,2)*s+MK(2,2)*xf)*(MC(1,1)*s+MC(2,1)*xf), r2*(MC(1,1)*s+MC(2,1)*xf)^2, r2*(MC(1,2)*s+MC(2,2)*xf)*(MC(1,1)*s+MC(2,1)*xf), -r2*(MFC(1)*s+MFC(2)*xf)*(MC(1,1)*s+MC(2,1)*xf);...
    r2*(MK(1,1)*s+MK(2,1)*xf)*(MC(1,2)*s+MC(2,2)*xf), r2*(MK(1,2)*s+MK(2,2)*xf)*(MC(1,2)*s+MC(2,2)*xf), r2*(MC(1,1)*s+MC(2,1)*xf)*(MC(1,2)*s+MC(2,2)*xf), r2*(MC(1,2)*s+MC(2,2)*xf)^2, -r2*(MFC(1)*s+MFC(2)*xf)*(MC(1,2)*s+MC(2,2)*xf);...
    -r2*(MK(1,1)*s+MK(2,1)*xf)*(MFC(1)*s+MFC(2)*xf), -r2*(MK(1,2)*s+MK(2,2)*xf)*(MFC(1)*s+MFC(2)*xf), -r2*(MC(1,1)*s+MC(2,1)*xf)*(MFC(1)*s+MFC(2)*xf), -r2*(MC(1,2)*s+MC(2,2)*xf)*(MFC(1)*s+MFC(2)*xf), r2*(MFC(1)*s+MFC(2)*xf)^2+q];
end

function g = designConstraint(xd, param)
s = xd(1);          % semi span  
c = xd(2);          % chord
e = eccentricity(xd);
g = zeros(9,1);
g(1) = s*c-15.001;
g(2) = 14.999-s*c;
g(3) = e-1;
g(4) = 0.1-e;
end

function dg = designConJacobian(xd, param)
dg = perturb_x(@(xd_) designConstraint(xd_, param), xd);
end

function g = rampStateCon(xd, x, param)
s = xd(1);          % semi span  
xf = flexural_axis(xd);
g = zeros(7,1);
g(1) = s*tan(x(1))+xf*tan(x(2))-1;
g(2) = -(s*tan(x(1))+xf*tan(x(2)))-1;
end

function g = roughStateCon(xd, x, param)
s = xd(1);          % semi span  
xf = flexural_axis(xd);
g = zeros(9,1);
g(1) = s*tan(x(1))+xf*tan(x(2))-1;
g(2) = -(s*tan(x(1))+xf*tan(x(2)))-1;
end


function [dg_d, dg] = rampConJacobian(xd, x, param)
dg_d = perturb_x(@(xd_) rampStateCon(xd_, x, param), xd);
dg = perturb_x(@(x_) rampStateCon(xd, x_, param), x);
end
function [dg_d, dg] = roughConJacobian(xd, x, param)
dg_d = perturb_x(@(xd_) roughStateCon(xd_, x, param), xd);
dg = perturb_x(@(x_) roughStateCon(xd, x_, param), x);
end
function J = obj_f_design_gradient(xu, x_d, t, param, input)
J = perturb_x(@(x_d) obj_f(xu, x_d, t, param,input), x_d)';
end
