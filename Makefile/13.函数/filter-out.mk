# 用法：$(filter-out _pattern, _text)

# 该函数用于从名字列表_text中根据模式_pattern滤除一部分名字，并将滤除后的列表返回。测试代码如下：

objects = main1.o foo.o main2.o bar.o 
result = $(filter-out main%.o, $(objects)) 
 
all: 
	@echo $(result) 