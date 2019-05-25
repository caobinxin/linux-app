# if 函数很像 GNU 的 make 所支持的条件语句——ifeq（参见前面所述的章节），if 函数的语法是：
# $(if <condition>,<then-part>)
# 或是
# $(if <condition>,<then-part>,<else-part>)
# 可见，if 函数可以包含“else”部分，或是不含。即 if 函数的参数可以是两个，也可以是三个。<condition>参数是 if 的表达式，如果其返回的为非空字符串，那么这个表达式
# 就相当于返回真，于是，<then-part>会被计算，否则<else-part>会被计算。而 if 函数的返回值是，如果<condition>为真（非空字符串），那个<then-part>会是整个函数的返回值，如果<condition>为假（空字符串），那么<else-part>会是整个函数的返回值，此时如果<else-part>没有被定义，那么，整个函数返回空字串。所以，<then-part>和<else-part>只会有一个被计算。


arg = 13

c = 13
b = 12

ifeq (12, $(arg))

a = $(if 1 , b)
aa = $(if , b)

else

a = $(if a, $b, $c) # 返回b变量的值
aa = $(if , $b, $c) # 返回 

endif

all:
	echo $(a) $(aa) 
