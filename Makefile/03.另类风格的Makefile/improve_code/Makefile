
objects = main.o add.o sub.o imul.o div.o

count: $(objects)
	gcc -o count $(objects)

main.o:main.h

add.o sub.o : ordinary.h

add.o:add.h 


sub.o:sub.h 

imul.0 div.o: float_point.h

imul.o:imul.h 


div.o:div.h


clean:
	rm $(objects) count 