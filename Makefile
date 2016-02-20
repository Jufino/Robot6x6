C=$(CROSS_COMPILE)gcc -Wall -g
CPP=$(CROSS_COMPILE)g++ -Wall -g

GFLAGS=`pkg-config --libs --cflags gtk+-2.0 gmodule-2.0`
OFLAGS=`pkg-config --libs --cflags opencv`

TARGET=robotTerenny

robotTerenny:robotTerenny.c
	$(CC) -c robotTerenny.c

install: $(TARGET)
	$(CROSS_COMPILE)ar rcs lib$(TARGET).a robotTerenny.o
	install lib$(TARGET).a /usr/lib/lib$(TARGET).a
	install *.h /usr/include/
	rm lib$(TARGET).a
	rm robotTerenny.o

clean:
	rm /usr/lib/lib$(TARGET).a
	rm /usr/include/$(TARGET).h

all: libsemafor.o main

libsemafor.o:semafor.c semafor.h
	$(C) -c semafor.c -o libsemafor.o
main:main.cpp
	$(CPP) main.cpp libsemafor.o -lrt -lpthread -lgpio -lserial -lrobotTerenny -o main $(OFLAGS)
