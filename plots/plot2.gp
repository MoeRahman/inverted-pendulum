load './plots/style.gp'

set terminal pngcairo size 640,540
set output 'plots/plot2.png'

set xlabel "Time (seconds)" textcolor rgb "white" font ",12"
set ylabel "Velocity (m/s)" textcolor rgb "white" font ",12"

plot 'pendulum_sim.csv' using 1:3 w l title 'Velocity',\
     ''                 using 1:9 w l title 'Velocity Estimate'

set output 
set terminal x11 1 title "Velocity & Velocity Estimate" persist
replot
