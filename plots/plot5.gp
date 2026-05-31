load './plots/style.gp'

set terminal pngcairo size 640,540
set output 'plots/plot5.png'

plot 'pendulum_sim.csv' using 1:12 w l title 'Angular Velocity', \
     ''                 using 1:13 w l title 'Angular Velocity Estimate'

set output
set terminal x11 1 title "Position Error [m]" persist
replot