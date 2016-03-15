default: build

BIN=tiny-path

all: build

clean:
	rm $(BIN) || true

build:
	gcc-4.9 -std=c99 -o tiny-path -O3 -fopenmp main.c
