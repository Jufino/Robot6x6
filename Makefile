CC=$(CROSS_COMPILE)gcc -g
TARGET=main

main:main.cpp
	$(CC) main.cpp -lgpio -o main
