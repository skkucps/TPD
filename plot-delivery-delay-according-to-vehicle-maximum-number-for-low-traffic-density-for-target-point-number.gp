#plot-delivery-delay-according-to-vehicle-maximum-number-for-low-traffic-density.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-delay-according-to-vehicle-maximum-number-for-target-point-number.eps"
set autoscale
set xrange [30:140]
#set xrange [100:200]
set yrange [0:3000]
set xlabel "Number of Vehicles[#vehicles]" font "Times-Roman,42"
set ylabel "Avg. Delivery Delay[sec]" font "Times-Roman,42"
#set xtic auto
set xtics 0,10
#set ytic auto
set ytics 0,500
set size 1.5,1.3
set pointsize 3
set key top right
#set boxwidth 0.4
plot "./result/stat-measure.xls" using 1:2 title 'TSF-1' with linespoints lw 5 pt 4, "./result/stat-measure.xls" using 1:3 title 'TSF-2' with linespoints lw 5 pt 3,   "./result/stat-measure.xls" using 1:4 title 'TSF-3' with linespoints lw 5 pt 6




	
