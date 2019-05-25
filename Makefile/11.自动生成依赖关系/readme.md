在 Makefile 中,我们的依赖关系可能会需要包含一系列的头文件,比如,如果我们的 main.c
中有一句“#include "defs.h"”,那么我们的依赖关系应该是:
main.o : main.c defs.h
但是,如果是一个比较大型的工程,你必需清楚哪些 C 文件包含了哪些头文件,并且,你
在加入或删除头文件时,也需要小心地修改 Makefile,这是一个很没有维护性的工作。为了
避免这种繁重而又容易出错的事情,我们可以使用 C/C++编译的一个功能。大多数的 C/C++编译器都支持一个“-M”的选项,即自动找寻源文件中包含的头文件,并生成一个依赖关
系。例如,如果我们执行下面的命令:
cc -M main.c
其输出是:
main.o : main.c defs.h
于是由编译器自动生成的依赖关系,这样一来,你就不必再手动书写若干文件的依赖关系,
而由编译器自动生成了。需要提醒一句的是,如果你使用 GNU 的 C/C++编译器,你得用
“-MM”参数,不然,“-M”参数会把一些标准库的头文件也包含进来。
gcc -M main.c 的输出是:
main.o: main.c defs.h /usr/include/stdio.h /usr/include/features.h \
/usr/include/sys/cdefs.h /usr/include/gnu/stubs.h \
/usr/lib/gcc-lib/i486-suse-linux/2.95.3/include/stddef.h \
/usr/include/bits/types.h /usr/include/bits/pthreadtypes.h \
/usr/include/bits/sched.h /usr/include/libio.h \
/usr/include/_G_config.h /usr/include/wchar.h \
/usr/include/bits/wchar.h /usr/include/gconv.h \
/usr/lib/gcc-lib/i486-suse-linux/2.95.3/include/stdarg.h \
/usr/include/bits/stdio_lim.h
gcc -MM main.c 的输出则是:
main.o: main.c defs.h




那么,编译器的这个功能如何与我们的 Makefile 联系在一起呢。因为这样一来,我们的
Makefile 也要根据这些源文件重新生成,让 Makefile 自已依赖于源文件?这个功能并不现
实,不过我们可以有其它手段来迂回地实现这一功能。GNU 组织建议把编译器为每一个源
文件的自动生成的依赖关系放到一个文件中,为每一个“name.c”的文件都生成一个“name.d”
的 Makefile 文件,[.d]文件中就存放对应[.c]文件的依赖关系。
于是,我们可以写出[.c]文件和[.d]文件的依赖关系,并让 make 自动更新或自成[.d]文件,并
把其包含在我们的主 Makefile 中,这样,我们就可以自动化地生成每个文件的依赖关系了。
这里,我们给出了一个模式规则来产生[.d]文件:









如果 make 执行时,带入 make 参数“-n”或“--just-print”,那么其只是显示命令,但不会执
行命令,这个功能很有利于我们调试我们的 Makefile,看看我们书写的命令是执行起来是什
么样子的或是什么顺序的。
而 make 参数“-s”或“--slient”则是全面禁止命令的显示。

