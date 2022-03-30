function dX = f(t, X, reference_type, adaptation_type)

% Parameters of system:
m = 4.25; % [kg]
d = 0.1; % [kg/sec]
g = 9.81; % [m/sec^2]
r = 0.26; % [m]
J = 0.0475; % [kg-m^2]
phi_bound = pi/3;

% Break up state:
q = X(1:4);
x = q(1); dx = q(2); y = q(3); dy = q(4);
theta = X(5); dtheta = X(6);

% Figure out which reference to use:
switch reference_type
    case 0 % Step
        [r, dr] = step_ref(t, 0, 0.0001, 1);
    case 1 % Ramp
        [r, dr] = ramp_ref(t, 0, 1);
    case 2 % Sinusoid
        [r, dr] = sine_ref(t, 1, 5, 0);
    case 3 % Custom trajectory 1
        [r, dr] = custom_periodic_ref_1(t, 0, 5);
end

% Figure out which adaptation law(s) to use:
switch adaptation_type
    case -2 % No control, pass reference input through
        u = r;
    case -1 % Static linear feedback
        [torque, phi] = static_SFB(q, r, dr);
    case 0 % Vanilla direct MRAC
        [torque, phi] = directMRAC(1,1);
    case 1 % Continuous deadzone-modification
        [torque, phi] = deadzone(1,1);
    case 2 % Sigma-modification
        [torque, phi] = sigmod(1,1);
    case 3 % e-modification
        [torque, phi] = emod(1,1);
    case 4 % Projection operator
        [torque, phi] = projection(1,1);
    case 5 % Nonlinear
        [torque, phi] = nonlinear(1,1);
end
% Saturate phi:
if ( abs(phi) > phi_bound )
    phi = sign(phi) * phi_bound;
end

% Equations of motion:
ddx = -(d/m)*dx - (torque/m)*sin(theta-phi);
ddy = -(d/m)*dy + (torque/m)*cos(theta-phi) - g;
ddtheta = -(r/J)*torque*sin(phi);
        
% Update derivative:        
dX = [dx; ddx; dy; ddy; dtheta; ddtheta];

end

