#plot-delivery-ratio-according-to-AP-maximum-number-for-reverse-forwarding.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-ratio-according-to-AP-maximum-number.eps"
set autoscale
#set xrange [50:140]
set xrange [1:13]
set yrange [0:1]
set xlabel "Number of APs[#APs]" font "Times-Roman,42"
set ylabel "Avg. Delivery Ratio" font "Times-Roman,42"
#set xtic auto
set xtics 0,1
#set ytic auto
set ytics 0,0.1
set size 1.5,1.3
set pointsize 3
set key bottom right
#set boxwidth 0.4
plot "./result/stat-delivery-ratio.xls" using 1:2 title 'TSF' with linespoints lw 5 pt 4, "./result/stat-delivery-ratio.xls" using 1:4 title 'RTP' with linespoints lw 5 pt 3,   "./result/stat-delivery-ratio.xls" using 1:3 title 'LTP' with linespoints lw 5 pt 6





	
