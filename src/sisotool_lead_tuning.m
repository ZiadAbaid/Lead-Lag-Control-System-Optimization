Gp = tf([658.5],[1 56 3136 0 0])
Glead = tf([1 1.0615] , [1 15.3]) ; 
Glag = tf([1 0.076] , [1 , 0.002]) ; 
Gc = Glag * Glead ; 
kc = 340;
Gt = Gp*Gc*kc ;
t = 0 : 0.01 : 10;
sisotool(Gt)

% 1. Define the Cost Function (The "Brain" of the study)
function Cost = myCostFunction(params)
    % Unpack parameters (The optimizer guesses these)
    Kc = params(1);
    Z_lead = params(2);
    % We fix Pole_lead to simplify, or you can optimize it too
    P_lead = 15.3; 
    
    % Define System
    s = tf('s');
    Plant = 658.56 / (s^2 * (s^2 + 56*s + 3136));
    
    % Construct Controller
    G_lead = Kc * (s + Z_lead) / (s + P_lead);
    G_lag = (s + 0.076) / (s + 0.002); % Keep Lag fixed for now
    
    % Open Loop
    L = G_lead * G_lag * Plant;
    
    % Get Margins
    [Gm, Pm, Wcg, Wcp] = margin(L);
    Gm_dB = 20*log10(Gm);
    
    % 2. Calculate Error (Eq. 5 from the paper)
    Target_PM = 55;   % Your target
    Target_GM = 20;   % Your target
    
    % Penalize instability heavily
    if Pm < 0 || Gm_dB < 0 
        Cost = 1e6;
    else
        Cost = (Pm - Target_PM)^2 + (Gm_dB - Target_GM)^2;
    end
end

% 3. Run Optimization (Like "fmincon" in the paper)
initial_guess = [200, 1]; 
optimized_params = fminsearch(@myCostFunction, initial_guess)
