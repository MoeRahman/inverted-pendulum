load './plots/style.gp'
set terminal x11 1 title "Position Error [m]" persist
plot 'pendulum_sim.csv' using 1:7 w l title 'Position Error'