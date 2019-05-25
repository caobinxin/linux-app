#include <stdio.h>
#include <stdlib.h>

int main(int argc, const char *argv[])
{
	char * value = getenv("LANG") ;
	printf("value = %s\n", value) ;
	
	printf("CBX_VAR=%s\n", getenv("CBX_VAR")) ;
	
	putenv("CBX_VAR=caobinxin") ;//添加环境变量
	printf("CBX_VAR=%s\n", getenv("CBX_VAR")) ;

	putenv("CBX_VAR=chenyang") ;//修改环境变量
	printf("CBX_VAR=%s\n", getenv("CBX_VAR")) ;
	
	/* 第三个参数，如果为0 环境变量已经存在时，不替换
	 *
	 * 				非0 时环境变量已经存在时替换*/
	setenv("CBX_VAR", "cao", 1) ;

	printf("CBX_VAR=%s\n", getenv("CBX_VAR")) ;
	return 0;
}
