#plot-cdf-of-delivery-delay.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-cdf-of-delivery-delay.eps"
set autoscale
set xrange [0:6000]
#set xrange [0:10000]
set yrange [0:1]
set xlabel "Delivery Delay[sec]" font "Times-Roman,42"
set ylabel "\% of Delay (CDF)" font "Times-Roman,42"
set xtic auto
#set xtics 0,100
#set ytic auto
set ytics 0,0.1
set size 1.5,1.3
set pointsize 3
set key bottom right
#set boxwidth 0.4
plot "./result/stat-cdf-of-tbd-for-add.xls" using 1:2 title 'TBD' with linespoints lw 5 pt 4, \
     "./result/stat-cdf-of-vadd-for-add.xls" using 1:2 title 'VADD' with linespoints lw 5 pt 6 



	