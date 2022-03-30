%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ECE6554: Adaptive Control
% Planar Ducted Fan Project
%   https://pvela.gatech.edu/classes/doku.php?id=ece6554:project_ductedfan
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear;
disp('Starting sim...');
file = fullfile(pwd, 'reference_trajectories');
addpath(file);
file = fullfile(pwd, 'adaptive_laws');
addpath(file);

%% Simulation parameters:
tEnd = 10; dt = 0.01;
tSpan = 0 : dt : tEnd; % Time span from 0 to tend seconds.

n_sims = 1;
reference_types = 0; % Reference input into closed-loop system.
adaptation_types = -1; % Adaptive law to be used.

%% Initial conditions:
x0 = 1; dx0 = 0; % x-coordinate of center-of-mass.
y0 = 5; dy0 = 0; % y-coordinate of center-of-mass.
theta0 = 0; dtheta0 = 0; % orientation of center-of-mass.
q0 = [x0; dx0; y0; dy0]; 
initial_conditions = [q0; theta0; dtheta0];

% Check simulation parameters and initial conditions are consistent:
do_run_sims = true;
if ( length(reference_types) ~= n_sims )
    fprintf(['Number of references (%i) ',...
            'does not match number of simulations to be ran (%i).\n'],...
            length(reference_types), n_sims);
    do_run_sims = false;
elseif ( length(adaptation_types) ~= n_sims )
    fprintf(['Number of adaptation types (%i) ',...
            'does not match number of simulations to be ran (%i).\n'],...
            length(adaptation_types), n_sims);
    do_run_sims = false;
elseif ( (length(x0) ~= n_sims) || (length(dx0) ~= n_sims) ...
        || (length(y0) ~= n_sims) || (length(dy0) ~= n_sims) ...
        || (length(theta0) ~= n_sims) || (length(dtheta0) ~= n_sims) )
    fprintf(['Number of initial conditions ',...
            'does not match number of simulations to be ran (%i).\n'],...
            n_sims);
    do_run_sims = false;
end

%% Run simulations:
if ( do_run_sims )
    for ii = 1:n_sims
        initial_condition = initial_conditions(:, ii);
        ref_type = reference_types(ii);
        adapt_type = adaptation_types(ii);
        [t, X] = ode45( @(t,X)planar_ducted_fan_dynamics(t, X,...
                            ref_type, adapt_type),...
                    tSpan, initial_condition);
                
        %% Unpack states:
        q = X(:, 1:4);
        x = q(:, 1); dx = q(:, 2);
        y = q(:, 3); dy = q(:, 4);
        theta = X(:, 5); dtheta = X(:, 6);

        %% Plot relevant data:
        parametric_plot_1 = figure('Name', 'Parametric Plot of x vs. y'); grid on;
            plot(x, y);
            xlabel('x'); ylabel('y'); title('Parametric Plot of x vs. y');
    end
end
             

