#plot-delivery-cost-according-to-AP-maximum-number-for-reverse-forwarding.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-cost-according-to-AP-maximum-number.eps"
set autoscale
set xrange [1:13]
set yrange [0:50]
set xlabel "Number of APs[#APs]" font "Times-Roman,42"
set ylabel "Avg. Delivery Cost" font "Times-Roman,42"
#set xtic auto
set xtics 0,1
#set ytic auto
set ytics 0,10
set size 1.5,1.3
set pointsize 3
set key top right
#set boxwidth 0.4
plot "./result/stat-cost-measure.xls" using 1:2 title 'TSF' with linespoints lw 5 pt 4, "./result/stat-cost-measure.xls" using 1:4 title 'RTP' with linespoints lw 5 pt 3,   "./result/stat-cost-measure.xls" using 1:3 title 'LTP' with linespoints lw 5 pt 6





	
