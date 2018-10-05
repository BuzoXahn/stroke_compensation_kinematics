#!/usr/bin/gnuplot -persist
#set terminal png
#set output 'rtask.png'

set xlabel "X"
set ylabel "Y"
set grid

set size 1,1
set origin 0,0

plot [-10:10][0:10] "rtask.dat" using 2:3 title 'Trajectory' with linespoints
# EOF
