#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#define UNIX_DOMAIN "UNIX_domain"
#define BUF_SIZE 1024
int main( void)
{
	int com_fd ;
	int ret ;
	char snd_buf[BUF_SIZE] ;
	int i ;
	int num ;

	static struct sockaddr_un srv_addr ;
	static struct sockaddr_un clt_addr ;
	char clt_file[] = "foo.socket" ;

	srv_addr.sun_family = AF_UNIX ;
	strcpy(srv_addr.sun_path, UNIX_DOMAIN) ;

	com_fd = socket( PF_UNIX, SOCK_DGRAM, 0) ;
	if(com_fd < 0)
	{
		perror("cannot create communication socket") ;
		return 1 ;
	}

	/**
	 * 保证文件名的唯一性，　以解决多客户端双向通信问题
	 * */
	mkstemp( clt_file) ;
	clt_addr.sun_family = AF_UNIX ;
	strcpy( clt_addr.sun_path, clt_file) ;
	unlink( clt_file) ;

	ret = bind( com_fd, (struct sockaddr*)&clt_addr, sizeof(clt_addr.sun_family) + strlen(clt_addr.sun_path)) ;
	if( -1 == ret)
	{
		perror("cannot bind server socket") ;
		return 1 ;
	}

	memset( snd_buf, 0, BUF_SIZE) ;
	sprintf( snd_buf, "%ld : message form client", (long)getpid()) ;
	printf(" snd_buf: %s\n", snd_buf) ;

	/**
	 * 	将信息发给服务器
	 * */

	for( i = 0; i < 4; i++)
	{
		sleep(1) ;
		num = sendto( com_fd, snd_buf, sizeof(snd_buf), 0, (struct sockaddr*)&srv_addr, sizeof(struct sockaddr)) ;	
		printf("send %d characters\n", num) ;
	}

	close( com_fd) ;
	unlink( clt_file) ;
	return 0 ;
}
