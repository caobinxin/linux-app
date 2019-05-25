# 用法：$(abspath _names)

# 该函数主要用于将_names中的各路径转换成绝对路径，并将转换后的结果返回。测试代码如下

# 添加绝对路径
ROOT := $(abspath prebuilts/gcc/linux-x86/host/x86_64-linux-glibc2.11-4.6/bin)
 
all:
	echo $(ROOT)   