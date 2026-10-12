#!/bin/sh
# Usage: mic_plot.sh [csv] [--once]
# Redraws data/mic_plot.png from the newest data/mic_*.csv every second (Ctrl+C to stop).
# Shows the last 20 s: level, peak, DC offset, and sound direction / wake state.
here=$(dirname "$0")
rows=400   # 20 s at 20 rows/s
csv=""; once=0
for a in "$@"; do
  case "$a" in --once) once=1 ;; *) csv="$a" ;; esac
done
[ -n "$csv" ] || csv=$(ls -t data/mic_*.csv 2>/dev/null | head -1)
[ -n "$csv" ] || { echo "No data/mic_*.csv yet. Run 'make log' first."; exit 1; }
out=data/mic_plot.png
win=data/.mic_window.csv

draw() {
  grep -E '^[0-9]+,' "$csv" | tail -n "$rows" > "$win"
  [ -s "$win" ] || return 0
  gnuplot -c "$here/mic_plot.gp" "$win" "$out"
}

if [ "$once" = 1 ]; then draw; echo "Plot: $out (from $csv)"; exit 0; fi
trap 'rm -f "$win"; echo; echo "Last plot: $out (from $csv)"' EXIT
trap 'exit 130' INT TERM HUP
echo "Plotting $csv -> $out every second (Ctrl+C to stop). Open $out in your editor to watch it."
while true; do draw; sleep 1; done
