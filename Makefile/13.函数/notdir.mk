# 用法：$(notdir _names)

# 该函数用于从路径_names中抽取文件名，并将文件名返回。测试代码如下

file_name := $(notdir code/foo/src/foo.c code/bar/src/bar.c)
 
all:
	@echo $(file_name)