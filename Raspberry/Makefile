C=$(CROSS_COMPILE)gcc -Wall -g
CPP=$(CROSS_COMPILE)g++ -Wall -g

GFLAGS=`pkg-config --libs --cflags gtk+-2.0 gmodule-2.0`
OFLAGS=`pkg-config --libs --cflags opencv`
LDFLAGS=-L/home/pi/Kinect/libfreenect/build/lib/libfreenect.so.0.5 -lfreenect -L/home/pi/Kinect/libusb-1.0.20/ -lusb-1.0 


all:main

main:main.cpp
	$(CPP) main.cpp $(LDFLAGS) -lrobotTerenny -lgpio -lserial -lm -lrt -lpthread -o main $(OFLAGS)

TARGET=robotTerenny

robotTerenny:robotTerenny.c
	$(CPP) -c robotTerenny.c $(LDFLAGS) -lgpio -lserial -lm -lrt -lpthread $(OFLAGS)

install: $(TARGET)
	$(CROSS_COMPILE)ar rcs lib$(TARGET).a robotTerenny.o
	install lib$(TARGET).a /usr/lib/lib$(TARGET).a
	install $(TARGET).h /usr/include/
	rm lib$(TARGET).a
	rm robotTerenny.o

libclean:
	rm /usr/lib/lib$(TARGET).a
	rm /usr/include/$(TARGET).h
clean:
	rm -f main
