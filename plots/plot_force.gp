load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/force.png'
plot 'pendulum_sim.csv' using 1:5 w l title 'Input Force - [N]'

set output
set terminal x11 2 title "Force" persist
replot
