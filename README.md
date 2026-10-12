# Closed-Loop Control System Modeling & Lead-Lag Compensator Optimization

**Tools:** MATLAB, Control System Toolbox, SISO Design Tool (`sisotool`), Simulink  
**Course:** Systems & Control — German University in Cairo (GUC)  

---

## Technical Overview

This repository implements the mathematical modeling, frequency-domain stabilization, and multi-objective **Nelder-Mead Simplex (`fminsearch`) optimization** of a Lead-Lag compensator for an unstable Type-2 fourth-order electromechanical plant:

$$G_p(s) = \frac{658.56}{s^2 (s^2 + 56s + 3136)}$$

### Compensator Architecture

To simultaneously stabilize the double-integrator plant, shape the transient step/ramp/parabolic response, and satisfy strict steady-state parabolic tracking error requirements, a cascaded **Lead-Lag controller** $G_c(s)$ is synthesized:

$$G_c(s) = K_c \cdot \underbrace{\left(\frac{s + z_{\text{lead}}}{s + p_{\text{lead}}}\right)}_{G_{\text{lead}}(s),\; p_{\text{lead}} > z_{\text{lead}}} \cdot \underbrace{\left(\frac{s + z_{\text{lag}}}{s + p_{\text{lag}}}\right)}_{G_{\text{lag}}(s),\; z_{\text{lag}} > p_{\text{lag}}}$$

### Multi-Objective Nelder-Mead Simplex Optimization (`src/lead_lag_simplex_optimizer.m`)

Rather than relying solely on trial-and-error Bode shaping, the 5-dimensional parameter vector $\mathbf{\theta} = [K_c,\; z_{\text{lead}},\; p_{\text{lead}},\; z_{\text{lag}},\; p_{\text{lag}}]^T$ is optimized via constrained Nelder-Mead Simplex search (`fminsearch`) minimizing a weighted quadratic penalty function over the open-loop transfer function $L(s) = G_c(s)G_p(s)$:

$$J(\mathbf{\theta}) = w_1 \left(\text{PM}(\mathbf{\theta}) - 55^\circ\right)^2 + w_2 \left(\text{GM}_{\text{dB}}(\mathbf{\theta}) - 20\text{ dB}\right)^2 + w_3 \left(K_a(\mathbf{\theta}) - 120\right)^2 + \Phi_{\text{stability}}(\mathbf{\theta})$$

where:
* **Target Phase Margin ($\text{PM}$):** $55^\circ$ (ensuring fast rise time with well-damped transient overshoot)
* **Target Gain Margin ($\text{GM}$):** $20\text{ dB}$ (robustness against plant parameter variations)
* **Target Acceleration Error Constant ($K_a = \lim_{s \to 0} s^2 L(s)$):** $120\text{ s}^{-2}$ (bounding steady-state quadratic acceleration tracking error $e_{\text{ss,quad}} = 1/K_a < 1\%$)
* **Structural & Hurwitz Barrier ($\Phi_{\text{stability}}$):** Enforces $\mathbf{\theta} > 0$, $p_{\text{lead}} > z_{\text{lead}}$, $z_{\text{lag}} > p_{\text{lag}}$, and positive stability margins.

---

## Repository Structure

* [`src/lead_lag_simplex_optimizer.m`](src/lead_lag_simplex_optimizer.m) — 5-variable Nelder-Mead Simplex optimizer (`fminsearch`) with structural Lead/Lag pole-zero constraints and automated margin/error verification.
* [`src/closed_loop_trajectory_analysis.m`](src/closed_loop_trajectory_analysis.m) — Time-domain simulation (`lsim`) comparing uncompensated vs. compensated closed-loop tracking across **Step (Position)**, **Ramp (Velocity)**, and **Quadratic (Acceleration)** reference inputs, plus closed-loop pole-zero maps (`pzmap`).
* [`src/sisotool_lead_tuning.m`](src/sisotool_lead_tuning.m) — Reduced-order Lead compensator tuning script integrated with MATLAB `sisotool` root-locus/Bode interactive design.
* [`src/compensator_verification.m`](src/compensator_verification.m) — Baseline unity-feedback closed-loop transfer function construction script.
