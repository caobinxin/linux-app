
objects = main.o add.o sub.o imul.o div.o

count: $(objects)
	gcc -o count $(objects)

main.o:main.h


add.o:add.h ordinary.h


sub.o:sub.h ordinary.h


imul.o:imul.h float_point.h


div.o:div.h float_point.h


clean:
	rm $(objects) count 


# 这种风格,让我们的 makefile 变得很简单,但我们的文件依赖关系就显得有点凌乱了。鱼和
# 熊掌不可兼得。还看你的喜好了。我是不喜欢这种风格的,一是文件的依赖关系看不清楚,
# 二是如果文件一多,要加入几个新的.o 文件,那就理不清楚了。