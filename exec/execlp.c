#include <stdio.h>
#include <unistd.h>

int main(int argc, const char *argv[])
{
	if( argc < 2)
	{
		printf("Usage: %s path\n", argv[0]) ;
		return 1 ;
	}

	execlp("ls", "caobinxin", argv[1], (char *) NULL) ;

	return 0;
}
