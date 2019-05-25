# 用法：$(eval _text)

# eval函数的存在使得Makefile具有动态语言的特征。eval函数使得make将再一次解析_text语句。测试如下所示

sources = foo.c bar.c baz.s ugh.h cao.cc bin.bin caobinxin.c
$(eval sources := $(filter %.c %.s, $(sources)))
 
all:
	@echo $(sources)