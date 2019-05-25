“伪目标”并不是一个文件,只是一个标签,由于
“伪目标”不是文件,所以 make 无法生成它的依赖关系和决定它是否要执行。我们只有通
过显示地指明这个“目标”才能让其生效。当然,“伪目标”的取名不能和文件名重名,不
然其就失去了“伪目标”的意义了。
当然,为了避免和文件重名的这种情况,我们可以使用一个特殊的标记“.PHONY”来显示
地指明一个目标是“伪目标”,向 make 说明,不管是否有这个文件,这个目标就是“伪目
标”。

.PHONY : clean
只要有这个声明,不管是否有“clean”文件,要运行“clean”这个目标,只有“make clean”
这样。于是整个过程可以这样写:
.PHONY: clean
clean:
rm *.o temp

伪目标一般没有依赖的文件。但是,我们也可以为伪目标指定所依赖的文件。伪目标同样可
以作为“默认目标”,只要将其放在第一个。一个示例就是,如果你的 Makefile 需要一口气
生成若干个可执行文件,但你只想简单地敲一个 make 完事,并且,所有的目标文件都写在
一个 Makefile 中,那么你可以使用“伪目标”这个特性:
all : prog1 prog2 prog3
.PHONY : all
prog1 : prog1.o utils.o
cc -o prog1 prog1.o utils.o
prog2 : prog2.o
cc -o prog2 prog2.o
prog3 : prog3.o sort.o utils.o
cc -o prog3 prog3.o sort.o utils.o
我们知道,Makefile 中的第一个目标会被作为其默认目标。我们声明了一个“all”的伪目标,
其依赖于其它三个目标。由于伪目标的特性是,总是被执行的,所以其依赖的那三个目标就
总是不如“all”这个目标新。所以,其它三个目标的规则总是会被决议。也就达到了我们一
口气生成多个目标的目的。“.PHONY : all”声明了“all”这个目标为“伪目标”。
随便提一句,从上面的例子我们可以看出,目标也可以成为依赖。所以,伪目标同样也可成
为依赖。看下面的例子:
.PHONY: cleanall cleanobj cleandiff
cleanall : cleanobj cleandiff
rm program
cleanobj :
rm *.o
cleandiff :
rm *.diff“make clean”将清除所有要被清除的文件。
“cleanobj”和“cleandiff”这两个伪目标有点像
“子程序”的意思。我们可以输入“make cleanall”和“make cleanobj”和“make cleandiff”
命令来达到清除不同种类文件的目的。