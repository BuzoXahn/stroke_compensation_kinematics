#!/usr/bin/gnuplot -persist
#set terminal png
#set output 'rtask.png'

set xlabel "Time (msec)"
set ylabel "Position"
set grid

set size 1,1
set origin 0,0
set multiplot

set size 1,0.5
set origin 0,0.5
plot [][-10:10] "rtask.dat" using 1:2 title 'X-position' with linespoints

set size 1,0.5
set origin 0,0
plot [][0:10] "rtask.dat" using 1:3 title "Y-position" with linespoints

# EOF
