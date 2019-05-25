objects_c= add.c sub.c
objects_o= add.o sub.o

include $(objects_c:.c=.d)#将.c文件替换成.d
include sub.d

# $$$$:随机数
%.d:%.c
	echo "$(CFLAG)"
	@echo "createing $*.d ..."
	@set -e;rm -f $@;\
	gcc $(CFLAG) -MM $< > $@.$$$$;\
	sed 's,\($*\).o[:]*,\1.o $@ : ,g' < $@.$$$$ > $@;\
	rm -f $@.$$$$
	#直接输出到制定目录下面
	echo "	gcc -c $(CFLAG) -o $(OUT_DIR)$*.o $*.c" >> $@
