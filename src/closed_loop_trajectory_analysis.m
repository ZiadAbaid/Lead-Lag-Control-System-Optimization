%% 1. System Definition
% Plant
Gp = tf([658.5], [1 56 3136 0 0]);

% Compensator Design
Glead = tf([1 0.93], [1 15.3]); 
Glag = tf([1 0.076], [1 0.002]); 
Gc = Glag * Glead; 
kc = 380;

% Create Closed Loop Systems
% BEFORE: Feedback with just the Plant (Warning: Likely Unstable)
Sys_Before = feedback(Gp, 1); 

% AFTER: Feedback with Compensator + Gain
Gt = Gp * Gc * kc;
Sys_After = feedback(Gt, 1);

%% 2. Simulation Setup
% Time Vector
t = 0:0.01:100; % Reduced to 100s for visibility (1000s might hide details)

% Define Inputs
u_step = ones(size(t));      % Step (Position)
u_ramp = t;                  % Ramp (Velocity)
u_quad = 0.5 * t.^2;         % Quadratic (Acceleration)

%% 3. Simulation & Plotting

figure('Name', 'Comparison: Before vs After Compensation');

% --- Analysis 1: Step Input ---
subplot(3, 2, 1);
[y_step_old, ~] = lsim(Sys_Before, u_step, t);
plot(t, u_step, 'k--', t, y_step_old, 'r');
title('Step Response (Before)'); grid on; legend('Input','Output');

subplot(3, 2, 2);
[y_step_new, ~] = lsim(Sys_After, u_step, t);
plot(t, u_step, 'k--', t, y_step_new, 'b');
title('Step Response (After)'); grid on; legend('Input','Output');
ylim([0 1.5]); % Zoom in to see the stable response

% --- Analysis 2: Ramp Input ---
subplot(3, 2, 3);
[y_ramp_old, ~] = lsim(Sys_Before, u_ramp, t);
plot(t, u_ramp, 'k--', t, y_ramp_old, 'r');
title('Ramp Response (Before)'); grid on;

subplot(3, 2, 4);
[y_ramp_new, ~] = lsim(Sys_After, u_ramp, t);
plot(t, u_ramp, 'k--', t, y_ramp_new, 'b');
title('Ramp Response (After)'); grid on;

% --- Analysis 3: Quadratic Input ---
subplot(3, 2, 5);
[y_quad_old, ~] = lsim(Sys_Before, u_quad, t);
plot(t, u_quad, 'k--', t, y_quad_old, 'r');
title('Quadratic Response (Before)'); grid on;

subplot(3, 2, 6);
[y_quad_new, ~] = lsim(Sys_After, u_quad, t);
plot(t, u_quad, 'k--', t, y_quad_new, 'b');
title('Quadratic Response (After)'); grid on;

%% 4. Error Calculation and Printing

fprintf('========================================\n');
fprintf('       STEADY STATE ERROR ANALYSIS      \n');
fprintf('========================================\n');

% --- STEP ERRORS ---
err_step_old = abs(u_step(end) - y_step_old(end));
err_step_new = abs(u_step(end) - y_step_new(end));
fprintf('STEP INPUT (Position):\n');
fprintf('  Before: %.5f (System is likely Unstable)\n', err_step_old);
fprintf('  After:  %.5f\n', err_step_new);
fprintf('----------------------------------------\n');

% --- RAMP ERRORS ---
err_ramp_old = abs(u_ramp(end) - y_ramp_old(end));
err_ramp_new = abs(u_ramp(end) - y_ramp_new(end));
fprintf('RAMP INPUT (Velocity):\n');
fprintf('  Before: %.5f\n', err_ramp_old);
fprintf('  After:  %.5f\n', err_ramp_new);
fprintf('----------------------------------------\n');

% --- QUADRATIC ERRORS ---
err_quad_old = abs(u_quad(end) - y_quad_old(end));
err_quad_new = abs(u_quad(end) - y_quad_new(end));
fprintf('QUADRATIC INPUT (Acceleration):\n');
fprintf('  Before: %.5f\n', err_quad_old);
fprintf('  After:  %.5f\n', err_quad_new);
fprintf('========================================\n');

figure(2) ; 
pzmap(Sys_Before);
figure(3) ; 
pzmap(Sys_After)