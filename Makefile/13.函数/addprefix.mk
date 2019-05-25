# 用法：$(addprefix _prefix, _names)

# 该函数主要用于给名字列表_names中的每一个名字增加前缀_prefix，并将增加了前缀的名字列表返回。测试代码如下

without_dir = main.c bar.c foo.c
with_dir := $(addprefix src/, $(without_dir))
 
all:
	@echo $(with_dir)