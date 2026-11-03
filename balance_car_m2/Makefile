CC ?= cc
CPPFLAGS ?= -Iinclude
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Wpedantic
LDFLAGS ?=

TARGETS := car_web
CAR_WEB_OBJECTS := src/car_web.o src/car_control.o src/uart.o

.PHONY: all clean

all: $(TARGETS)

car_web: $(CAR_WEB_OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $(CAR_WEB_OBJECTS)

src/uart.o: include/uart.h
src/car_control.o: include/car_control.h include/uart.h
src/car_web.o: include/car_control.h

src/%.o: src/%.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(CAR_WEB_OBJECTS) $(TARGETS)
