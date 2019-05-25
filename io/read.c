#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define BUF_SIZE  20
int main(int argc, const char *argv[])
{
	int fd ;
	char buffer[BUF_SIZE] ;

	int num ;

	if( argc != 2)
	{
		printf("Usage: %s filename", argv[0]) ;
		return 1 ;
	}

	if( (fd = open( argv[1], O_RDONLY)) == -1)
	{
		perror("Cannot open the file") ;
		return 1 ;
	}

	while( ( num = read(fd, buffer, BUF_SIZE - 1)) > 0)
	{
		buffer[num] = '\0' ;
		printf("%s", buffer) ;
	}

	printf("num = %d\n", num) ;
	
	close( fd) ;

	return 0;
}
