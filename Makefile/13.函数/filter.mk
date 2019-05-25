# 用法：$(filter _pattern, _text)

# 该函数用于从一个名字列表_text中根据模式_pattern得到满足需要的名字列表返回。测试代码如下

sources = foo.c bar.c baz.s ugh.h cao.cc bin.bin
sources := $(filter %.c %.s, $(sources))
 
all:
	@echo $(sources)