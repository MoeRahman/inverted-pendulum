cmake --build build
./build/inverted_pendulum
gnuplot -p -e "set grid; set datafile separator ','; plot 'data.csv' using 1:2 w l title 'X-Position', '' using 1:3 w l title 'Pole Angle' "
gnuplot -p -e "set grid; set datafile separator ','; plot 'data.csv' using 1:4 w l title 'Voltage'"
