set title "Visit Count Heat Map"
unset key
set tic scale 0
set palette rgbformula -7,2,-7
set cbrange [0:609980]
set cblabel "Visit Count"
unset cbtics

set xrange [-0.5:6.5]
set yrange [-0.5:6.5]
set view map

pause 0.01
splot '/share_home/taehwan.hwang/Simulation/TPD/TPD-v1.9.2/result.txt' matrix with image  
