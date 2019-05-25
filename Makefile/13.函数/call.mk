#  call函数是唯一一个可以用来创建新的参数化的函数。你可以写一个非常复杂的表达式，这个表达式中，你可以定义许多参数，然后你可以用call函数来向这个表达式传递参数。其语法是：

# $(call <expression>;,<parm1>;,<parm2>;,<parm3>;...)


# 当make执行这个函数时，<expression>;参数中的变量，如$(1)，$(2)，$(3)等，会被参数< parm1>;，<parm2>;，<parm3>;依次取代。而<expression>;的返回值就是 call函数的返回值。例如：

reverseab =  $(1) $(2)

fooab = $(call reverseab,a,b) #那么，foo的值就是“a b”。当然，参数的次序是可以自定义的，不一定是顺序的，如：



reverseba =  $(2) $(1)

fooba = $(call reverseba,a,b) #此时的foo的值就是“b a”。

all:
	echo $(fooab) $(fooba)

