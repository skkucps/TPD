#plot-delivery-delay-according-to-vehicle-maximum-number-for-low-traffic-density.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-delay-according-to-vehicle-maximum-number-for-low-traffic-density.eps"
set autoscale
#set xrange [50:140]
set xrange [40:300]
set yrange [600:2000]
set xlabel "Number of Vehicles[#vehicles]" font "Times-Roman,42"
set ylabel "Avg. Delivery Delay[sec]" font "Times-Roman,42"
#set xtic auto
#set xtics 0,10
set xtics 0,40
#set ytic auto
set ytics 0,200
set size 1.5,1.3
set pointsize 3
set key top right
#set boxwidth 0.4
plot "./result/stat-measure.xls" using 1:2 title 'ADAPTIVE' with linespoints lw 5 pt 4, \
     "./result/stat-measure.xls" using 1:3 title 'STATIC' with linespoints lw 5 pt 6


#     "./result/stat-measure.xls" using 1:3 title 'STATIC' with linespoints lw 5 pt 6, \
#     "./result/stat-measure.xls" using 1:4 title 'DYNAMIC' with linespoints lw 5 pt 3



	