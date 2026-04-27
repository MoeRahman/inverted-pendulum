time cmake --build build
./build/inverted_pendulum

gnuplot -p -e " set grid; 
                set datafile separator ','; 
                set grid lc rgb 'white';
                set border lc rgb 'white';
                set key tc rgb 'white';
                set terminal x11 0 title "Simulation" persist;
                set object 1 rectangle from screen 0,0 to screen 1,1 fillcolor rgb 'black' behind;
                plot 'pendulum_sim.csv' using 1:2 w l title 'X-Position', ''
                using 1:4 w l title 'Pole Angle', '' 
                using 1:6 w l title 'Position Setpoint'"

gnuplot -p -e " set grid; 
                set datafile separator ','; 
                set grid lc rgb 'white';
                set border lc rgb 'white';
                set key tc rgb 'white';
                set terminal x11 0 title "Noise" persist;
                set object 1 rectangle from screen 0,0 to screen 1,1 fillcolor rgb 'black' behind;
                plot 'pendulum_sim.csv' using 1:7 title 'Noise'"

gnuplot -p -e " set grid; 
                set datafile separator ',';
                set grid lc rgb 'white';
                set border lc rgb 'white';
                set key tc rgb 'white';
                set terminal x11 0 title "Force" persist;
                set object 1 rectangle from screen 0,0 to screen 1,1 fillcolor rgb 'black' behind; 
                plot 'pendulum_sim.csv' using 1:5 w l title 'Input Force - [N]'"
