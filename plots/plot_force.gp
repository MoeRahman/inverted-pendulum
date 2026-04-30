load './plots/style.gp'
set terminal x11 2 title "Force" persist
plot 'pendulum_sim.csv' using 1:5 w l title 'Input Force - [N]'
