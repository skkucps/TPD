#plot-delivery-delay-according-to-AP-maximum-number.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-delay-according-to-AP-maximum-number.eps"
set autoscale
#set xrange [50:140]
set xrange [1:9]
set yrange [0:3500]
set xlabel "Number of APs[#APs]" font "Times-Roman,42"
set ylabel "Avg. Delivery Delay[sec]" font "Times-Roman,42"
#set xtic auto
#set xtics 0,10
#set ytic auto
set ytics 0,500
set size 1.5,1.3
set pointsize 3
set key top right
#set boxwidth 0.4
plot "./result/stat-measure.xls" using 1:2 title 'TBD' with linespoints lw 5 pt 4, \
     "./result/stat-measure.xls" using 1:3 title 'VADD' with linespoints lw 5 pt 6




	
