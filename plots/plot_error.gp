load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'error.png'
plot 'pendulum_sim.csv' using 1:7 w l title 'Position Error'

set output
set terminal x11 1 title "Position Error [m]" persist
replot