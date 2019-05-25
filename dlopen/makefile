all:
	gcc -fPIC -shared caculate.c -o libcaculate.so
	gcc -rdynamic -o main.bin main.c -ldl

clean:
	rm *.bin  *.so