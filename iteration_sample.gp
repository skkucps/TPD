reset
#set term gif animate
#set output "animate.gif"
n=24 #n frames
dt=2*pi/n
set xrange [0:4*pi]
set yrange [-1:1]
do for [i=0:n]{
  plot sin(x+i*dt)/(1. + i/12.) w l lw 1.5 title sprintf("t=%i",i)
  pause 0.1
}
#set output
