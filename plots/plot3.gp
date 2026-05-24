load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/force.png'
plot 'pendulum_sim.csv' using 1:2 w l title 'Position', \
     ''                 using 1:7 w l title 'Position Error', \
     ''                 using 1:8 w l dt 2 title 'Position Estimate'

set output
set terminal x11 2 title "Position" persist
replot
