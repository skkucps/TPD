#plot-delivery-delay-according-to-SN-maximum-number-for-reverse-forwarding.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-delay-according-to-RN-maximum-number.eps"
set autoscale
set xrange [31:49]
set yrange [0:3000]
set xlabel "Number of RNs[#RNs]" font "Times-Roman,42"
set ylabel "Avg. Delivery Delay[sec]" font "Times-Roman,42"
#set xtic auto
set xtics 1,2
#set ytic auto
set ytics 0,500
set size 1.5,1.3
set pointsize 3
set key top right
#set boxwidth 0.4
plot "./result/stat-measure.xls" using 1:2 title 'TSF' with linespoints lw 5 pt 4, "./result/stat-measure.xls" using 1:4 title 'RTP' with linespoints lw 5 pt 3,   "./result/stat-measure.xls" using 1:3 title 'LTP' with linespoints lw 5 pt 6





	
