cmake --build build
./build/inverted_pendulum

gnuplot -p -e " set grid; 
                set datafile separator ','; 
                set grid lc rgb 'white';
                set border lc rgb 'white';
                set key tc rgb 'white';
                set object 1 rectangle from screen 0,0 to screen 1,1 fillcolor rgb 'black' behind;
                plot 'data.csv' using 1:2 w l title 'X-Position', '' 
                using 1:3 w l title 'Pole Angle', '' 
                using 1:5 w l title 'Position Setpoint'"

gnuplot -p -e " set grid; 
                set datafile separator ',';
                set grid lc rgb 'white';
                set border lc rgb 'white';
                set key tc rgb 'white';
                set object 1 rectangle from screen 0,0 to screen 1,1 fillcolor rgb 'black' behind; 
                plot 'data.csv' using 1:4 w l title 'Voltage'"
