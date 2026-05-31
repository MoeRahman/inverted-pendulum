load './plots/style.gp'

set terminal pngcairo size 640,540
set output 'plots/plot6.png'

plot 'pendulum_sim.csv' using 1:5 w l title 'Force Input'

set output
set terminal x11 1 title "Force [N]" persist
replot