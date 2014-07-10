#plot-link-delay-standard-deviation-for-vehicle-interarrival-time.gp

set terminal postscript eps 35
set output "./figure/link-delay-standard-deviation-comparison-for-one-way-traffic.eps"
set autoscale
set xrange [0:20]
set yrange [0:100]
set xlabel "Vehicle Inter-arrival Time[sec]" font "Times-Roman,42"
set ylabel "Link Delay Standard Deviation[sec]" font "Times-Roman,42"
#set xtic auto
set xtics 0,2
#set ytic auto
set ytics 0,10
set size 1.5,1.3
set pointsize 3
set key top right
#set boxwidth 0.4
plot "./result/stat-sd-measure.xls" using 1:2 title 'Simulation' with linespoints lw 5 pt 3, \
     "./result/stat-sd-estimate.xls" using 1:2 title 'TBD' with linespoints lw 5 pt 4


	