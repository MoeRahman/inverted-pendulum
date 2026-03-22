# Project Overview:
The simulation of the classic invertered pendulum controls problem. The hope is to increase complexity in the controller and system design to improve fidelity of the state-space model and design more sophisticated controllers around the model.

## Tentitive Plan:

__Phase I - Single Pendulum Simulation:__ *Equations of motion · Euler/RK4 integrator · Visualization · PID angle then cascaded cart+angle*

__Phase II - STM32 Model Build:__ *Encoder + IMU sensing · Motor driver · Real-time loop · Port PID from sim*

__Phase III - Sim/Real Comparison + Sim Fidelity:__ *Add friction · Encoder quantization · Motor deadband · Kalman / EKF noise estimation*

__Phase IV - Advanced Control On Single Pendulum:__ *LQR · Pole placement · Full state feedback · Lyapunov stability · swing-up energy method*

__Phase V - Double Pendulum Simulation & Hardware:__ *Lagrangian derivation · Chaos sensitivity · UKF state estimation · LQR near equilibrium*

__Phase VI - Data-Driven Double Pendulum:__ *RL (PPO/SAC) · MPC · Generalized moment / Beneš filter comparison*

__Phase VII - Triple Pendulum Simulation & Hardware:__ *Higher-order Lagrangian · Mechanical design challenge · Structural resonance*

__Phase VIII - Deep-Learning & Transformers On Triple:__ *LSTM policy · Decision transformer · World model · Sim-to-real transfer*

__Phase IX - Benchmarking:__ *Estimator benchmarks · Control hierarchy comparison · Literature gaps*

## Updates:
Updates on the project here.
### March 22, 2026
