load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/state_estimate.png'
plot 'pendulum_sim.csv' using 1:2 w l title 'Position', \
     ''                 using 1:8 w l title 'Position Estimate'

set output
set terminal x11 1 title "Position Error [m]" persist
replot