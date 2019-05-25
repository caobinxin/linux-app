# 用法：$(wildcard _pattern)

# 该函数是通配符函数，通过它可以得到当前工作目录中满足_pattern模式的文件或目录名列表，测试代码如下

SRCS = $(wildcard *.mk)
 
all:
	@echo $(SRCS)