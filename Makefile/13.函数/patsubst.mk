# 用法：$(patsubst _pattern, _replacement, _text)

# 该函数用于将名字列表_text中符合_pattern模式的名字替换为_replacement，并将替换后的名字列表返回。测试代码如下：
mixed = foo.c bar.c main.o
 
objects = $(patsubst %.c, %.o, $(mixed))
 
all:
	@echo $(objects)

#当然也可以采用变量赋值的高级用法，即在赋值的时候，完成文件名后缀替换工作，如下所示 
mixed = foo.c bar.c main.o
 
objects = $(mixed:.c=.o)
 
new_all:
	@echo $(objects)