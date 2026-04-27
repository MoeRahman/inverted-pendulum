load 'style.gp'
set terminal x11 1 title "Velocity" persist
plot 'pendulum_sim.csv' using 1:3 w l title 'Velocity'
