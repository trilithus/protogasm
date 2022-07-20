#Motor vs Pressure
set datafile separator ","
set y2tics
set ytics nomirror
plot 'data.bin' \
   using 1:2 title 'Motor' with lines axis x1y1, \
'' using 1:3 title 'Pressure' with lines axis x1y2, \
'' using 1:4 title 'Avg Pressure' with lines axis x1y2
pause mouse close