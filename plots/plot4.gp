load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/plot4.png'

set xlabel "Time (seconds)" textcolor rgb "white" font ",12"
set ylabel "Angle (degrees)" textcolor rgb "white" font ",12"

plot 'pendulum_sim.csv' using 1:4 w l title 'Angle', \
     ''                 using 1:10 w l title 'Angle Estimate'

set output
set terminal x11 1 title "Angle vs. Angle Estimate" persist
replot