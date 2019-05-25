
#变量相当于是宏，所以这里并不展开,要想让统配符在变量中展开，需要使用wildcard关键字
#var = *.o
var := $(wildcard *.o)
objects =  main.o add.o sub.o imul.o div.o

#编译命令也可以和依赖写在同一行
count: $(objects);gcc -o count $(objects)
	

#类似c中的include 会在这里进行原封不动的包含
# include ordinary.mk \
#                   float_point.mk

#用下面的语句替代这个include

#这个紧紧是重新改写了默认的规则　并没有替代掉　include　mk的功能，并没有指示清楚每个文件的依赖
$(filter %.o, $(objects)): %.o : %.c
	echo "依赖是:$<    目标是:$@"
	gcc -c $< -o $@

main.o:main.h

.PHONY : clean
clean:
	-rm *.o count 