set xrange[0:8]
set yrange[0:8]
set grid
set title "TPD"
unset key
plot 'vehicle_point.txt' using 2:3:1 with labels
