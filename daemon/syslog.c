#include <stdio.h>
#include <syslog.h>

int main(int argc, const char *argv[])
{
	openlog(argv[0], LOG_CONS | LOG_PID, LOG_USER) ;

	int count = 0 ;
	while( count < 5)
	{
		syslog(LOG_INFO, "%d, log info test ...", count) ;
		count++ ;
	}

	closelog() ;

	return 0;
}
