#plot-delivery-delay-according-to-vehicle-speed-deviation.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-delay-according-to-vehicle-speed-deviation.eps"
set autoscale
set xrange [1:10]
#set yrange [300:800]
set yrange [600:2000]
set xlabel "Vehicle Speed Deviation[MPH]" font "Times-Roman,42"
set ylabel "Avg. Delivery Delay[sec]" font "Times-Roman,42"
#set xtic auto
set xtics 0,1
#set ytic auto
set ytics 0,200
set size 1.5,1.3
set pointsize 3
set key top right
#set boxwidth 0.4
plot "./result/stat-measure.xls" using 1:2 title 'ADAPTIVE' with linespoints lw 5 pt 4, \
     "./result/stat-measure.xls" using 1:3 title 'STATIC' with linespoints lw 5 pt 6



	