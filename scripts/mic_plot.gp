# gnuplot -c mic_plot.gp <window.csv> <png>
csv = ARG1
out = ARG2

set terminal png size 1600,1000 font "Helvetica,12"
set output out
set datafile separator ','

# time axis: seconds relative to the newest row (0 = now)
stats csv using 1 nooutput
t_end = STATS_max
t(x) = (x - t_end) / 1000.0

set grid
set xrange [-20:0]
set xlabel "Time (s, 0 = now)"
set key top left
blue = "#1f77b4"; red = "#d62728"; green = "#2ca02c"

set multiplot layout 2,2 title "Toy robot microphones (last 20 s)" font "Helvetica,15"

# Healthy: about -60 dBFS or lower in a quiet room, jumps 20-40 dB when you speak or clap
set title "Level (RMS, dBFS)"
set ylabel "dBFS"
set yrange [-100:0]
plot csv using (t($1)):2 with lines lw 2 lc rgb blue title "mic A (left)", \
     csv using (t($1)):3 with lines lw 2 lc rgb red title "mic B (right)", \
     -45 with lines dt 2 lc rgb "gray40" title "too loud for a quiet room"

# Full scale is 32767; flat-topped near it means clipping (lower the gain shift in menuconfig)
set title "Peak (16-bit counts)"
set ylabel "counts"
set yrange [0:33000]
plot csv using (t($1)):4 with lines lw 2 lc rgb blue title "mic A", \
     csv using (t($1)):5 with lines lw 2 lc rgb red title "mic B"

# Should sit near a steady value; wandering or huge values point at wiring noise
set title "DC offset (16-bit counts)"
set ylabel "counts"
set yrange [*:*]
plot csv using (t($1)):6 with lines lw 2 lc rgb blue title "mic A", \
     csv using (t($1)):7 with lines lw 2 lc rgb red title "mic B"

set key top right
# Difference between the mics; a steady offset or drift points at one mic being closer, covered, or wired differently
set title "Channel balance (mic A - mic B, dB)"
set ylabel "dB"
set yrange [-20:20]
plot csv using (t($1)):($2 - $3) with lines lw 2 lc rgb blue title "A - B", \
     csv using (t($1)):($8 * 15) with lines lw 2 lc rgb green title "mic -> speaker loop on"

unset multiplot
