load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/simulation.png'
plot 'pendulum_sim.csv' using 1:11 w p pt "." pointsize 0.1 title 'Position Measurement', \
     ''                 using 1:6 w l title 'Position Setpoint', \
     ''                 using 1:2 w l title 'Position'

set output 
set terminal x11 0 title "Simulation" persist
replot