load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/force.png'
plot 'pendulum_sim.csv' using 1:2 w l lw 2 title 'Position', \
     ''                 using 1:8 w l lw 2 dt 3 title 'Position Estimate', \
     ''                 using 1:11 w l lw 2 dt 3 title 'Position Measurement'

set output
set terminal x11 2 title "Position" persist
replot
