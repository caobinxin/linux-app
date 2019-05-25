make 命令开始时,会把找寻 include 所指出的其它 Makefile,并把其内容安置在当前的位置。
就好像 C/C++的#include 指令一样。如果文件都没有指定绝对路径或是相对路径的话,make
会在当前目录下首先寻找,如果当前目录下没有找到,那么,make 还会在下面的几个目录
下找:
1、如果 make 执行时,有“-I”或“--include-dir”参数,那么 make 就会在这个参数所
指定的目录下去寻找。
2、如果目录<prefix>/include(一般是:/usr/local/bin 或/usr/include)存在的话,make
也会去找。
如果有文件没有找到的话,make 会生成一条警告信息,但不会马上出现致命错误。它会继
续载入其它的文件,一旦完成 makefile 的读取,make 会再重试这些没有找到,或是不能读
取的文件,如果还是不行,make 才会出现一条致命信息。如果你想让 make 不理那些无法
读取的文件,而继续执行,你可以在 include 前加一个减号“-”。如:
-include <filename>
其表示,无论 include 过程中出现什么错误,都不要报错继续执行。和其它版本 make
兼容的相关命令是 sinclude,其作用和这一个是一样的。