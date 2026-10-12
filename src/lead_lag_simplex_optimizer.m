%% --- 1. MAIN SCRIPT ---
clc; clear; close all;

% Initial Guess: [Kc, Z_lead, P_lead, Z_lag, P_lag]
% We use your previous manual values as a starting point so the optimizer doesn't get lost.
% Order:   [ Gain,  Lead_Z, Lead_P, Lag_Z,  Lag_P ]
x0 =       [ 200,   1,   7,   0.4, 0.005 ]; 

% Run Optimization (Nelder-Mead Simplex Method)
% 'MaxFunEvals' increased because we have 5 variables to tune
options = optimset('MaxFunEvals', 2000, 'MaxIter', 2000);
optimized_params = fminsearch(@myCostFunction, x0, options);

% --- DISPLAY RESULTS ---
Kc_opt      = optimized_params(1);
Z_lead_opt  = optimized_params(2);
P_lead_opt  = optimized_params(3);
Z_lag_opt   = optimized_params(4);
P_lag_opt   = optimized_params(5);

fprintf('==============================================\n');
fprintf('       FULL COMPENSATOR PREDICTION RESULTS    \n');
fprintf('==============================================\n');
fprintf('Compensator Gain (Kc) : %.4f\n', Kc_opt);
fprintf('Lead Zero (Z_lead)    : %.4f\n', Z_lead_opt);
fprintf('Lead Pole (P_lead)    : %.4f\n', P_lead_opt);
fprintf('Lag Zero  (Z_lag)     : %.4f\n', Z_lag_opt);
fprintf('Lag Pole  (P_lag)     : %.4f\n', P_lag_opt);
fprintf('----------------------------------------------\n');

% --- VERIFICATION ---
s = tf('s');
Plant = 658.56 / (s^2 * (s^2 + 56*s + 3136));
G_lead = Kc_opt * (s + Z_lead_opt) / (s + P_lead_opt);
G_lag  = (s + Z_lag_opt) / (s + P_lag_opt);
L_opt  = G_lead * G_lag * Plant;

[Gm, Pm, ~, ~] = margin(L_opt);
Gm_dB = 20*log10(Gm);

% Calculate Ka (Acceleration Constant)
% Ka = lim s^2 * L(s)
% For Type 2, the s^2 cancels, leaving just the DC gains
Plant_DC_Gain = 658.56 / 3136; 
Comp_DC_Gain  = Kc_opt * (Z_lead_opt/P_lead_opt) * (Z_lag_opt/P_lag_opt);
Ka_actual = Comp_DC_Gain * Plant_DC_Gain;

fprintf('VERIFIED PERFORMANCE:\n');
fprintf('  Phase Margin : %.2f deg  (Target: 55)\n', Pm);
fprintf('  Gain Margin  : %.2f dB   (Target: 20)\n', Gm_dB);
fprintf('  Ka Constant  : %.2f      (Target: 120)\n', Ka_actual);
fprintf('==============================================\n');


%% --- 2. COST FUNCTION ---
function Cost = myCostFunction(params)
    % Unpack
    Kc     = params(1);
    Z_lead = params(2);
    P_lead = params(3);
    Z_lag  = params(4);
    P_lag  = params(5);
    
    % --- CONSTRAINTS (Penalties) ---
    % 1. All values must be positive (Stable poles/zeros)
    if any(params <= 0)
        Cost = 1e10; return;
    end
    
    % 2. Structure Constraints (Optional but helpful)
    % Lead Pole must be > Lead Zero (to actually be a Lead)
    % Lag Zero must be > Lag Pole (to actually be a Lag)
    if P_lead < Z_lead || Z_lag < P_lag
         Cost = 1e10; return;
    end
    
    % Define System
    s = tf('s');
    Plant = 658.56 / (s^2 * (s^2 + 56*s + 3136));
    
    % Compensator
    G_lead = Kc * (s + Z_lead) / (s + P_lead);
    G_lag  = (s + Z_lag) / (s + P_lag);
    
    L = G_lead * G_lag * Plant;
    
    % Get Margins
    [Gm, Pm, ~, ~] = margin(L);
    Gm_dB = 20*log10(Gm);
    
    % Get Ka (Acceleration Constant)
    Plant_DC = 658.56 / 3136;
    Comp_DC  = Kc * (Z_lead/P_lead) * (Z_lag/P_lag);
    Ka = Comp_DC * Plant_DC;
    
    % --- TARGETS ---
    Target_PM = 55;
    Target_GM = 20;
    Target_Ka = 120; % Corresponds to < 1% error
    
    % --- COST CALCULATION ---
    % We weight the errors slightly differently to balance them
    Cost = 1.0 * (Pm - Target_PM)^2 ...
         + 1.0 * (Gm_dB - Target_GM)^2 ...
         + 0.05 * (Ka - Target_Ka)^2; % Lower weight on Ka since it can be large
     
    % Penalty for instability
    if Pm <= 0 || Gm_dB <= 0 || isnan(Pm)
        Cost = 1e10;
    end
end