
objects = main.o add.o sub.o imul.o div.o

#编译命令也可以和依赖写在同一行
count: $(objects);gcc -o count $(objects)
	

#类似c中的include 会在这里进行原封不动的包含
include ordinary.mk \
                  float_point.mk

main.o:main.h

.PHONY : clean
clean:
	-rm $(objects) count 