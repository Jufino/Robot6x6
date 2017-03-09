C=$(CROSS_COMPILE)gcc -Wall -g
CPP=$(CROSS_COMPILE)g++ -Wall -g

GFLAGS=`pkg-config --libs --cflags gtk+-2.0 gmodule-2.0`
OFLAGS=`pkg-config --libs --cflags opencv`

all:
	sudo make install
	sudo rm main
	sudo make main

main:main.cpp
	$(CPP) -I/usr/local/include/libfreenect main.cpp -lrobotTerenny -lgpio -lusb-1.0  -lm -lrt -lpthread -o main $(OFLAGS)

TARGET=robotTerenny

robotTerenny:robotTerenny.c
	$(CPP) -I/usr/local/include/libfreenect -c robotTerenny.c -lgpio -lusb-1.0 -lm -lrt -lpthread $(OFLAGS)

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
