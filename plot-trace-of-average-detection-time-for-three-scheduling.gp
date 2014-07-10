#plot-trace-of-average-detection-time-for-three-scheduling.gp

set terminal postscript eps 35
set output "./figure/trace-of-adt-for-three-scheduling.eps"
set autoscale
#set xrange [0.01:5]
set yrange [0:40]
set xlabel "Duration [hours]" font "Times-Roman,44"
set ylabel "Avg. Detection Time [sec]" font "Times-Roman,44"
set xtic auto
#set xtics 0,1
set ytic auto
#set size 1.5,1.3
set size 1.55,1.3
set pointsize 3
set key top right
#set boxwidth 0.4

#@set line styles
set linestyle 1 lt 1 lw 5
set linestyle 2 lt 2 lw 5
set linestyle 3 lt 5 lw 5

#
plot "./result/stat-trace-of-adt-for-virtual-scan.xls" using ($1/3600):2 title 'Virtual Scan' with l ls 1, \
     "./result/stat-trace-of-adt-for-duty-cycling.xls" using ($1/3600):2 title 'Duty Cycling' with l ls 2, \
     "./result/stat-trace-of-adt-for-always-awake.xls" using ($1/3600):2 title 'Always-Awake' with linespoints lw 5 pt 3
	