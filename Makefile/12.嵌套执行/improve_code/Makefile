
CFLAG= -I "/home/colby/work/daily-demo/linux_app/Makefile/12.嵌套执行/improve_code/include"
export CFLAG
OUT_DIR= "/home/colby/work/daily-demo/linux_app/Makefile/12.嵌套执行/improve_code/out/"
export OUT_DIR

all:
	$(MAKE) -C ./main
	$(MAKE) -C ./add
	$(MAKE) -C ./imul


.PHONY : clean
clean:
	-rm *.o count *.d

exec:
	cd ../
	pwd
execc:
	cd ../;pwd

count: $(objects_o);gcc -o count $(CFLAG) $(objects_o)