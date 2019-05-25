#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stddef.h>

#define UNIX_DOMAIN "UNIX.domain"
#define BUF_SIZE 1024

int main(int argc, const char *argv[])
{
	socklen_t addr_len ;
	int listen_fd ;
	int com_fd ;
	int ret ;
	int i ;
	static char recv_buf[BUF_SIZE] ;
	int len ;
	int num ;

	struct sockaddr_un clt_addr ;
	struct sockaddr_un srv_addr ;
	int tmpa_size, tmpb_size ;

	/**
	 * 创建通信用的套接字
	 */

	com_fd = socket( PF_UNIX, SOCK_DGRAM, 0) ;
	if( com_fd < 0)
	{
		perror("cannot create listening socket") ;
	}

	srv_addr.sun_family = AF_UNIX ;
	strcpy( srv_addr.sun_path, UNIX_DOMAIN) ;
	unlink(UNIX_DOMAIN) ;

	/**
	 * 将地址信息与套接字绑定
	 */

	tmpa_size = offsetof(struct sockaddr_un, sun_path) + strlen( srv_addr.sun_path) ;
	tmpb_size = sizeof(srv_addr.sun_family) + strlen(srv_addr.sun_path) ;
	if( tmpa_size == tmpb_size)
	{
		printf("相等\n") ;
	}else{
		printf("tmpa_size = %d\ntmpb_size = %d\n", tmpa_size, tmpb_size) ;
	}

	ret = bind( com_fd, (struct sockaddr*)&srv_addr, sizeof(srv_addr.sun_family) + strlen(srv_addr.sun_path)) ;
	if( -1 == ret)
	{
		perror("cannot bind server socket") ;
		close( com_fd) ;
		unlink( UNIX_DOMAIN) ;
		return 1 ;
	}

	/**
	 * 读取客户端发送过来的信息并输出
	 */
	for( i = 0; i < 4; i++)
	{
		memset( recv_buf, 0, BUF_SIZE) ;
		printf("recvfrom...\n") ;
		num = recvfrom( com_fd, recv_buf, BUF_SIZE, 0, (struct sockaddr*)&clt_addr, &addr_len) ; 
		printf("Message from client (%d) : %s\n", num, recv_buf) ;
	}

	close( com_fd) ;
	unlink( UNIX_DOMAIN) ;
	return 0;
}
