# 1. Output Configuration
set terminal pngcairo  size 1920, 1080
set output './plots/plot2.png'

# 2. Load Your Reusable Theme File
load './plots/styles.gp'

# 3. CSV File Configuration
set datafile separator comma

# 4. Labels and Layout
set title "Angle vs. Angle Estimated"
set xlabel "Time (s)"
set ylabel "Angle / Error"
set grid

# 5. Multi-Column Plotting (Using your column indices)
# 1:time 2:x 3.vel 4:angle 5:input 6:setpoint 7:pos_err 8:x_est 9:vel_est 10:angle_est 11:x_meas 12:ang_vel 13:ang_vel_est 14:vel_err 15:theta_err 16:ang_vel_err
plot './pendulum_sim.csv' using 1:4 title 'Angle' w l ls 1, \
     './pendulum_sim.csv' using 1:10 title 'Angle Estimate' w l ls 2, \
     './pendulum_sim.csv' using 1:15 title 'Angle Error' w l ls 5
