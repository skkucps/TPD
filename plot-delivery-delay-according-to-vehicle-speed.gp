#plot-delivery-delay-according-to-vehicle-speed.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-delay-according-to-vehicle-speed.eps"
set autoscale
set xrange [20:60]
#set yrange [300:1000]
#set yrange [300:800]
set yrange [0:4000]
set xlabel "Vehicle Speed[MPH]" font "Times-Roman,42"
set ylabel "Avg. Delivery Delay[sec]" font "Times-Roman,42"
#set xtic auto
set xtics 0,5
#set ytic auto
set ytics 0,500
set size 1.5,1.3
set pointsize 3
set key top right
#set boxwidth 0.4
plot "./result/stat-measure.xls" using 1:2 title 'TBD' with linespoints lw 5 pt 4, \
     "./result/stat-measure.xls" using 1:3 title 'VADD' with linespoints lw 5 pt 6

#     "./result/stat-measure.xls" using 1:3 title 'TBD EDD + VADD Link' with linespoints lw 5 pt 5, \
#     "./result/stat-measure.xls" using 1:5 title 'VADD EDD + TBD Link' with linespoints lw 5 pt 7, \


	