load './plots/style.gp'

set terminal pngcairo size 640,540
set output 'plots/plot3.png'

set xlabel "Time (seconds)" textcolor rgb "white" font ",12"
set ylabel "Position (meters)" textcolor rgb "white" font ",12"

plot 'pendulum_sim.csv' using 1:2 w l title 'Position', \
     ''                 using 1:7 w l lw 3 title 'Position Error', \
     ''                 using 1:8 w l dt 2 title 'Position Estimate'

set output
set terminal x11 2 title "Position" persist
replot
