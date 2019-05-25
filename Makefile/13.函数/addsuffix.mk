# 用法：$(addsuffix _suffix, _names)

# 该函数主要用于给名字列表_names中的每一个名字增加后缀_suffix，并将增加了后缀_suffix的名字列表返回。测试代码如下

without_suffix = main foo bar
with_suffix := $(addsuffix .c, $(without_suffix))
 
all:
	@echo $(with_suffix)