load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/state_estimate.png'
plot 'pendulum_sim.csv' using 1:12 w l title 'Angular Velocity', \
     ''                 using 1:13 w l title 'Angular Velocity Estimate'

set output
set terminal x11 1 title "Position Error [m]" persist
replot