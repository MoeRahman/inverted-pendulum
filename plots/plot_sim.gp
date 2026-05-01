load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/simulation.png'
plot 'pendulum_sim.csv' using 1:2 w l title 'X-Position', \
     ''                 using 1:4 w l title 'Pole Angle', \
     ''                 using 1:6 w l title 'Position Setpoint'

set output 
set terminal x11 0 title "Simulation" persist
replot