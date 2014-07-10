#plot-cdf-of-delivery-delay.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-cdf-of-delivery-delay.eps"
set autoscale
#set xrange [0:7000]
set xrange [0:2200]
set yrange [0:1]
set xlabel "Delivery Delay[sec]" font "Times-Roman,42"
set ylabel "\% of Delay (CDF)" font "Times-Roman,42"
set xtic auto
#set xtics 0,100
#set ytic auto
set ytics 0,0.1
set size 1.5,1.3
set pointsize 3
#set key bottom right
set key top left
#set boxwidth 0.4
plot "./result/stat-cdf-of-tsf-for-add.xls" using 1:2 title 'TSF' with linespoints lw 5 pt 4, "./result/stat-cdf-of-rtp-for-add.xls" using 1:2 title 'RTP' with linespoints lw 5 pt 3, "./result/stat-cdf-of-ltp-for-add.xls" using 1:2 title 'LTP' with linespoints lw 5 pt 6



	
