default: build

BIN=tiny-path

all: build

clean:
	rm $(BIN) || true

build:
	gcc -o tiny-path -O3 main.c
