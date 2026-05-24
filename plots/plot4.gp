load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/error.png'
plot 'pendulum_sim.csv' using 1:4 w l title 'Angle', \
     ''                 using 1:10 w l title 'Angle Estimate'

set output
set terminal x11 1 title "Noisy Position vs. True Position" persist
replot