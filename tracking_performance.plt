# set terminal gif animate delay 10
# set output 'tracking.gif'

set key autotitle columnhead
set datafile separator ','

set linetype 11 linecolor rgb "red"
set linetype 12 linecolor rgb "green"


# Plot trajectory and simulated movement
 plot 'trajectory.csv' using 'x':'y' with lines lt rgb "black", 'tracking.csv' using 'pos_x':'pos_y':(sqrt($3 * $3 + $4 * $4) < 0 ? 12 : 11) linecolor variable with lines

# velocity / time
# plot 'tracking.csv' using 'timestamp':(sqrt($3 * $3 + $4 * $4)) with lines

stats 'tracking.csv' nooutput

# do for [i=1:29927] {
# 	plot 'trajectory.csv' using 'x':'y' with lines lt rgb "black", 'tracking.csv' using 'pos_x':'pos_y' every ::1::i with lines lt rgb "red"
# }

exit