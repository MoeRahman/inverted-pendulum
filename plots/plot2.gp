# 1. Output Configuration
set terminal pngcairo  size 1920, 1080
set output './plots/plot2.png'

# 2. Load Your Reusable Theme File
load './plots/styles.gp'

# 3. Binary File Configuration
BIN_DATA = "'./sim_file.bin' binary format='%16lf'"

# 4. Labels and Layout
set title "Angle vs. Angle Estimated"
set xlabel "Time (s)"
set ylabel "Angle / Error"
set grid

# 5. Multi-Column Plotting (Using your column indices)
# 1:time 2:position 3:velocity 4:angle 5:angular_velocity 6:input 7:setpoint
# 8:pos_err 9:vel_err 10:angle_err 11:ang_vel_err
# 12:pos_est 13:vel_est 14:angle_est 15:ang_vel_est 16:pos_meas
plot @BIN_DATA using 1:4 title 'Angle' w l ls 1, \
     @BIN_DATA using 1:14 title 'Angle Estimate' w l ls 2, \
     @BIN_DATA using 1:10 title 'Angle Error' w l ls 5
