# Makefile for host-side unit tests (no STM32 hardware required)
#
# Usage:
#   cd Tests && make          # build and run
#   cd Tests && make clean    # remove artefacts

CC      = gcc
CFLAGS  = -Wall -Wextra -std=c11 -I../Core/Inc

SRC     = test_initial_sequence.c ../Core/Src/Autonomous_functions.c
BIN     = test_initial_sequence

.PHONY: all clean

all: $(BIN)
	./$(BIN)

$(BIN): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(BIN)
