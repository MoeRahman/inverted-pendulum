load './plots/style.gp'

set terminal pngcairo size 800,600
set output 'plots/plot1.png'

set xlabel "Time (seconds)" textcolor rgb "white" font ",12"
set ylabel "Position (meters)" textcolor rgb "white" font ",12"

plot 'pendulum_sim.csv' using 1:11 w p pt "." pointsize 0.1 title 'Position Measurement', \
     ''                 using 1:6 w l title 'Position Setpoint', \
     ''                 using 1:2 w l title 'Position' 

set output 
set terminal x11 0 title "True Postion[m], Position Measurement[m]" persist
replot