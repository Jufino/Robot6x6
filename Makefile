C=$(CROSS_COMPILE)gcc -Wall -g
CPP=$(CROSS_COMPILE)g++ -Wall -g

GFLAGS=`pkg-config --libs --cflags gtk+-2.0 gmodule-2.0`
OFLAGS=`pkg-config --libs --cflags opencv`

TARGET=main

all: libsemafor.o main

libsemafor.o:semafor.c semafor.h
	$(C) -c semafor.c -o libsemafor.o
main:main.cpp
	$(CPP) main.cpp -lrt -lpthread -lgpio -o main $(OFLAGS)
