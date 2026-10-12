# Thin wrapper around idf.py. Run `source scripts/env.fish` (or env.sh) first.
PORT ?=
BAUD ?= 2000000
IDF  := idf.py $(if $(PORT),-p $(PORT)) -b $(BAUD)

.PHONY: build flash app monitor run menuconfig size clean fullclean

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
