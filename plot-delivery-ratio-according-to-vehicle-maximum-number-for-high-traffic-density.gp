#plot-delivery-ratio-according-to-vehicle-maximum-number-for-high-traffic-density.gp

set terminal postscript eps 35
set output "./figure/performance-comparison-of-delivery-ratio-according-to-vehicle-maximum-number-for-high-traffic-density.eps"
set autoscale
set xrange [0:1000]
set yrange [0.5:1]
set xlabel "Number of Vehicles[#vehicles]" font "Times-Roman,42"
set ylabel "Avg. Delivery Ratio" font "Times-Roman,42"
#set xtic auto
set xtics 0,100
#set ytic auto
set ytics 0,0.1
set size 1.5,1.3
set pointsize 3
#set key top right
set key bottom right
#set boxwidth 0.4
plot "./result/stat-delivery-ratio.xls" using 1:2 title 'TBD' with linespoints lw 5 pt 4, \
     "./result/stat-delivery-ratio.xls" using 1:4 title 'VADD' with linespoints lw 5 pt 6

#     "./result/stat-measure.xls" using 1:3 title 'TBD EDD + VADD Link' with linespoints lw 5 pt 5, \
#     "./result/stat-measure.xls" using 1:5 title 'VADD EDD + TBD Link' with linespoints lw 5 pt 7, \


	