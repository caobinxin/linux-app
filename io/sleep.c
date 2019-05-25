#include <stdio.h>
#include <unistd.h>

int main(int argc, const char *argv[])
{
	int i = 0 ;
	while( i < 1000)
	{
		i++ ;
		sleep(1) ;
		printf("sleep %d printf\n", i) ;
	}
	return 0;
}
