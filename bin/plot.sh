#!/bin/sh
sed "s/rtask.dat/$1/g" ./sensor_test_plot.p | gnuplot -persist
sed "s/rtask.dat/$1/g" ./sensor_test_plot2.p | gnuplot -persist

