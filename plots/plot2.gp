load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/velocity.png'
plot 'pendulum_sim.csv' using 1:3 w l title 'Velocity',\
     ''                 using 1:9 w l title 'Velocity Estimate'

set output 
set terminal x11 1 title "Velocity & Velocity Estimate" persist
replot
