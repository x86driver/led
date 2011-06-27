TARGET = led
CC = gcc
CFLAGS = -Wall -g -lpthread

all:$(TARGET)

led:led.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -rf $(TARGET)
