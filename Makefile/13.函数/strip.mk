# 用法：$(strip _string)

# 用于清除名字列表中的多余空格，测试代码如下

original = foo.c          bar.c
stripped := $(strip $(original))
 
all:
	@echo $(stripped)