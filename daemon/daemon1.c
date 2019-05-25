#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int daemon( int nochdir, int noclose)
{
	pid_t pid ;

	pid = fork() ;

	if( pid < 0)
	{
		perror("fork") ;
		return -1 ;
	}

	if( pid != 0)
	{
		printf("当前是父进程(%d) 子进程的id(%d)\n", getpid(), pid) ;
		printf("父进程退出....\n") ;
		exit(0) ;
	}

	printf("当前为子进程(%d)\n", getpid()) ;
	pid = setsid() ;
	printf("脱离终端后的pid(%d)\n", pid) ;

	if( pid < -1)
	{
		perror("setsid") ;
		return -1 ;
	}

	if( ! nochdir) chdir("/") ;

	if( ! noclose)
	{
		int fd ;
		fd = open("/dev/null", O_RDWR, 0) ;
		if( fd != -1)
		{
			dup2( fd, STDIN_FILENO) ;
			dup2( fd, STDOUT_FILENO) ;
			dup2( fd, STDERR_FILENO) ;
			if ( fd > 2)
				close( fd) ;
		}
	}

	umask(0027) ;

	return 0 ;	
}

int main(int argc, const char *argv[])
{
	daemon( 0, 0) ;
	sleep(1000) ;

	return 0;
}
