objects_c=main.c
objects_o=main.o

include $(objects_c:.c=.d)#将.c文件替换成.d

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


# 	echo "	cp ./$*.o ../out/" >> $@
# 	echo "	rm -f ./$*.o" >> $@
# #建议不要删除当前目录下面的.o　这样就不用每次都重新编译

test:
	echo $(objects_c:.c=.d)