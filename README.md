# TubeMPC_SoC
Robust Tube-based MPC for SoC control of a BESS. 

# SoC control with Robust MPC
This notebook demonstrates how Model Predictive Control (MPC) performance is influenced by system disturbances and how robust approaches, such as tube-based MPC, can effectively mitigate these effects.  

A simple linear energy storage model is considered.

**Key features:**

* **Decision variables**: charge/discharge powers over the horizon. 
* **Objective function:** track the SoC reference (stage and terminal terms) and penalize input rate changes.  
* **Constraints:** SoC and power bounds, along with system dynamics.  
* **Receding horizon**: solve, apply the first control move, roll forward, repeat.

## Model Implementation
$\mathrm{SOC}_{k+1}=\mathrm{SOC}_k+\frac{\Delta t}{E}\!\left(\eta_c P^{\mathrm{ch}}_k-\frac{1}{\eta_d}P^{\mathrm{dis}}_k\right)$
<br>with $P^{ch}≥0,P^{dis}≥0$, and bounds on SoC and power.
<br>Where:

* $SOC$ is the State of Charge
* $\Delta t$ is the control interval [h]
* $E$ is the usable capacity [kWh]
* $\eta_c$ is the charge efficency
* $P^{\mathrm{ch}}_k$ is the charge power [kW]
* $\eta_d$ is the discharge efficency
* $P^{\mathrm{dis}}_k$ is the discharge power [kW]
* $P_k=P_k^{dis}−P_k^{ch}$ is the net power [kW]

## Controlled System with disturbances
<br>

$\mathrm{SOC}_{k+1}=\mathrm{SOC}_k+\frac{\Delta t}{E}\!\left(\eta_c P^{\mathrm{ch}}_k-\frac{1}{\eta_d}P^{\mathrm{dis}}_k\right) -\frac{\Delta t}{E}P_{\mathrm{dist},k} + w^{\mathrm{soc}}_k$

- Parasitic load $P_{\text{dist}}[k]$ (kW) is injected only in the plant update and MPC doesn’t see this in its prediction model, so tracking degrades
  <br>Positive $P_{\text{dist}}$ drains the battery (e.g., unknown auxiliary load).

- Process noise $w^{\text{soc}}_k$ is tiny random drift directly on SoC.

**Remark:** 
<br>An Autoregressive (AR) model describes the values of a random variable in terms of historical values. The order of an autoregression is the number of immediately preceding values in the series that are used to predict the value at the present time.

## Standard MPC
Build a MPC with receding horizon.
<br>
### Cost function
Given:
- $P^{\text{net}}_k = P^{\text{dis}}_k - P^{\text{ch}}_k$ (discharge-positive net power),
- weights $w_{\text{soc,stage}}, w_{\text{soc,term}}, w_{\text{power}}, w_{\text{slew}}, w_{\text{anti}}$,
- horizon length \(N\),
- previous applied net power $P_{\text{prev}}$,

the cost is

$\begin{aligned}J&=\underbrace{w_{\mathrm{soc,stage}}\sum_{k=0}^{N-1}\!\big(\mathrm{SOC}_k-\mathrm{SOC}^{\mathrm{ref}}_k\big)^2}_{\mathrm{track SoC over the horizon}}\;+\;\underbrace{w_{\mathrm{soc,term}}\big(\mathrm{SOC}_N-\mathrm{SOC}^{\mathrm{ref}}_N\big)^2}_{\mathrm{terminal SoC target}}\quad+\;\underbrace{w_{\mathrm{power}}\sum_{k=0}^{N-1}\!\big(P^{\mathrm{ch}}_k+P^{\mathrm{dis}}_k\big)^2}_{\mathrm{keep total charge/discharge flow small}}\\[4pt]&\quad+\;\underbrace{w_{\mathrm{slew}}\Big(P^{\mathrm{net}}_0-P_{\mathrm{prev}}\Big)^2\;+\;w_{\mathrm{slew}}\sum_{k=1}^{N-1}\!\big(P^{\mathrm{net}}_k-P^{\mathrm{net}}_{k-1}\big)^2}_{\mathrm{penalize input rate-of-change(slew)}}\quad+\;\underbrace{w_{\mathrm{anti}}\sum_{k=0}^{N-1}\!\big(P^{\mathrm{ch}}_kP^{\mathrm{dis}}_k\big)^2}_{\mathrm{discourage simultaneous charge and discharge (smooth)}}\end{aligned}$

**Term-by-term:**
- **Stage tracking** $w_{\text{soc,stage}}$: keeps $\text{SOC}_k$ close to $\text{SOC}^{\text{ref}}_k$ at each step.
- **Terminal tracking** $w_{\text{soc,term}}$: emphasizes hitting the end-of-horizon SoC target.
- **Power regularization** $w_{\text{power}}$: penalizes *total* throughput $(P^{\text{ch}}+P^{\text{dis}})$ to avoid unnecessary cycling and heating.
- **Slew penalty** $w_{\text{slew}}$: smooths commands by penalizing changes in $P^{\text{net}}$; the first term compares to the **previous applied** input $P_{\text{prev}}$ to avoid a jump at the first move.
- **Anti-simultaneity** $w_{\text{anti}}$: a smooth penalty on $P^{\text{ch}} P^{\text{dis}}$ that nudges the optimizer away from charging and discharging in the same sample.

**Reamrk:**
<br>Without the $(P^{\text{ch}}P^{\text{dis}})^2$ term the objective is quadratic; that product makes the NLP smooth but generally nonconvex (still handled well by IPOPT in practice).

## Tube-based MPC

In the tube formulation you optimize a nominal problem with a charge-positive input $u_k=P^{ch}_k-P^{dis}_k$ and a simplified linear model $x_{k+1}=x_k+bu_k$. You then apply the ancillary feedback $u=u^*+K(x^{true}-x^*)$ where $e=x-x^*$ is the error.

* Optimize a _nominal_ trajectory $(x^\star, u^\star)$ on **tightened constraints** and apply $u = u^\star + K\,(x^{\text{true}} - x^\star)$ with a fixed stabilizing gain $K$ (ancillary feedback).

* Tightening uses an **RPI $^1$ tube** $Z=[-z,+z]$ with radius $z = \bar w /(1 - |A + BK|)$ (scalar case, $A=1$), where $\bar w$ is the worst-case step disturbance in SoC units.

  Tightened sets:  
  $X_{\text{tight}} = [x_{\min}+z,\; x_{\max}-z]$,  
  $U_{\text{tight}} = [u_{\min}+|K|z,\; u_{\max}-|K|z]$.

Why this solution is robust:

* With $A=1$, $B=b$, the ancillary loop $x^+=(1+Kb)x+bu^*$ has closed-loop pole $\alpha = 1+Kb$. Picking $|\alpha| < 1$ makes the error contract, so the tube radius $z=\bar{w}/(1-|\alpha|)$ bounds tracking error.
* Nominal optimizer ensures $x^*\in X⊖Z$ and $u^*\in U⊖KZ$. Then the applied pair $(x,u)$ stays in the original constraints $X,U$ for any disturbance $|w_k|\le \bar{w}$ provided you don’t hit actuator saturation.

The “tube” in this example is constant because a time-invariant tube is used: we precompute one invariant error bound $𝑍=[−𝑧,+𝑧]$ that contains all future errors under bounded disturbance and ancillary feedback. This is conservative at the early steps, because the error actually starts small (often and only grows toward its steady bound).

#### Remarks
$^1$ A robust positive invariant (RPI) set is a region in a dynamic system's state space such that if a system's state starts within this set, its future states will always remain within the set, even in the presence of disturbances. 
<br> $^2$ Increase alpha toward 1.0 for weaker feedback (i.e. small K) &#8594; larger tube &#8594; More conservative &#8594; may lead to slow response or infeasibility.

### Cost function
Differences vs the non-robust cost:
* There’s no $P^{ch}$, $P^{dis}$ inside the optimizer—just one input $u$. (The physical split $P^{ch}=max(u,0)$, $P^{dis}=max(-u,0)$ happens in the plant when we apply control.)
* No separate anti-simultaneity term is needed because a single $u$ can’t “charge and discharge at once.”
* Everything is evaluated on the nominal trajectory subject to tightened constraints; the ancillary feedback handles disturbances outside the optimizer.

## Resourches:
D. Pozo, _Linear battery models for power systems analysis_, Electric Power Systems Research, 212, 2022.
<br>J. B. Rawlings, D. Q. Mayne, and M. M. Diehl, _Model Predictive Control: Theory, Computation, and Design_, Nob Hill Publishing, Santa Barbara, CA, 2nd, paperback edition, 2020
<br>D.Q. Mayne, M.M. Seron and S.V. Raković, _Robust model predictive control of constrained linear systems with bounded disturbances_, Automatica, 41, 2, 2005