# Thin wrapper around idf.py. Run `source scripts/env.fish` (or env.sh) first.
PORT ?=
BAUD ?= 2000000
IDF  := idf.py $(if $(PORT),-p $(PORT)) -b $(BAUD)

.PHONY: build flash app monitor run menuconfig size clean fullclean log plot

build:
	$(IDF) build

flash:
	$(IDF) flash

app:
	$(IDF) app-flash

monitor:
	$(IDF) monitor

run:
	$(IDF) app-flash monitor

menuconfig:
	$(IDF) menuconfig

size:
	$(IDF) size-components

clean:
	$(IDF) clean

fullclean:
	$(IDF) fullclean

# Mic debugging: `make log` replaces `make monitor` (same console, plus saves mic levels to data/mic_*.csv);
# run `make plot` in a second terminal to watch them as a graph in data/mic_plot.png
log:
	python scripts/mic_log.py $(PORT)

plot:
	sh scripts/mic_plot.sh $(CSV)
