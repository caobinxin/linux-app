count: main.o add.o sub.o imul.o div.o
	gcc -o count main.o add.o sub.o imul.o div.o

main.o: main.c main.h
	gcc -c main.c 

add.o: add.c add.h
	gcc -c add.c 

sub.o: sub.c sub.h
	gcc -c sub.c 

imul.o: imul.c imul.h
	gcc -c imul.c 

div.o: div.c div.h
	gcc -c div.c 

clean:
	rm *.o count