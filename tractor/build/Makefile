#!bin/bash
main:main.o serial_open.o gps_com.o navigation.o ctrl_com.o
	gcc -o main main.o serial_open.o gps_com.o navigation.o ctrl_com.o -lm
main.o:../main.c
	gcc -c ../main.c -lm
serial_open.o:../include/serial_open.c ../include/serial_open.h
	gcc -c ../include/serial_open.c -lm
gps_com.o:../include/gps_com.c ../include/gps_com.h ../include/serial_open.h
	gcc -c ../include/gps_com.c
navigation.o:../include/navigation.c ../include/navigation.h ../include/serial_open.h
	gcc -c ../include/navigation.c
ctrl_com.o:../include/ctrl_com.c ../include/ctrl_com.h ../include/serial_open.h
	gcc -c ../include/ctrl_com.c

.PHONY:clean
clean:
	rm *.o

