#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void do_at_exit_one( void)
{
	printf("1.do_at_exit\n") ;
}

void do_at_exit_two( void)
{
	printf("2.do_at_exit\n") ;
}

int main(int argc, const char *argv[])
{
	int flag ;

	flag = atexit( do_at_exit_one) ;
	if( flag != 0)
	{
		printf("Cannot set exit function \n") ;
		return EXIT_FAILURE ;
	}


	flag = atexit( do_at_exit_two) ;
	if( flag != 0)
	{
		printf("Cannot set exit function \n") ;
		return EXIT_FAILURE ;
	}

	/*退出时，不会 调用 atexit 注册的函数*/
	_exit(EXIT_SUCCESS) ; 
}
