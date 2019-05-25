
#变量相当于是宏，所以这里并不展开,要想让统配符在变量中展开，需要使用wildcard关键字
#var = *.o
var:=$(wildcard *.o)
objects_c=main.c add.c sub.c imul.c div.c
objects_o=main.o add.o sub.o imul.o div.o



count: $(objects_o);gcc -o count $(objects_o)

include $(objects_c:.c=.d)#将.c文件替换成.d

# $$$$:随机数
%.d:%.c
	@echo "createing $*.d ..."
	@set -e;rm -f $@;\
	gcc -MM $< > $@.$$$$;\
	sed 's,\($*\).o[:]*,\1.o $@ : ,g' < $@.$$$$ > $@;\
	rm -f $@.$$$$

.PHONY : clean
clean:
	-rm *.o count *.d

exec:
	cd ../
	pwd
execc:
	cd ../;pwd