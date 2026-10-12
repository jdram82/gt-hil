# gt-hil - gas turbine HIL, Tier 0/1 (standard Linux)
#
#   make            build the shared library and the unit-test binary
#   make unit       run the C unit tests
#   make scenarios  run all scenarios and write reports to results/
#   make test       unit tests + all scenarios (the one command to run)
#   make clean

CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=c99 -fPIC -Iinclude
LDLIBS  := -lm

LIB_SRC := plant/plant.c plant/sim.c controller/standin_governor.c controller/sensor_vote.c controller/protection.c
LIB     := build/libgtsim.so
TESTBIN := build/test_plant
TESTCTL := build/test_controller

.PHONY: all unit scenarios test clean

all: $(LIB) $(TESTBIN) $(TESTCTL)

build:
	mkdir -p build

$(LIB): $(LIB_SRC) plant/plant.h plant/sim.h controller/standin_governor.h controller/sensor_vote.h controller/protection.h include/hil_constants.h include/hil_regs.h | build
	$(CC) $(CFLAGS) -shared -o $@ $(LIB_SRC) $(LDLIBS)

$(TESTBIN): tests/test_plant.c $(LIB_SRC) plant/plant.h plant/sim.h controller/standin_governor.h controller/sensor_vote.h controller/protection.h include/hil_constants.h include/hil_regs.h | build
	$(CC) $(CFLAGS) -o $@ tests/test_plant.c $(LIB_SRC) $(LDLIBS)

$(TESTCTL): tests/test_controller.c $(LIB_SRC) plant/plant.h plant/sim.h controller/standin_governor.h controller/sensor_vote.h controller/protection.h include/hil_constants.h include/hil_regs.h | build
	$(CC) $(CFLAGS) -o $@ tests/test_controller.c $(LIB_SRC) $(LDLIBS)

unit: $(TESTBIN) $(TESTCTL)
	./$(TESTBIN)
	./$(TESTCTL)

scenarios: $(LIB)
	python3 tools/scenario_runner.py scenarios/*.json

test: unit scenarios

clean:
	rm -rf build results/*
