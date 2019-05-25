#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pcap/pcap.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


pcap_t *source_pcap_t=NULL;
pcap_dumper_t *des_pcap_dumper_t=NULL;
 
int exit_main()
{
	printf("exit_main() is called.\n");
	if( NULL!=source_pcap_t )
	{
		pcap_close(source_pcap_t);
	}
	if( NULL!=des_pcap_dumper_t )
	{
		pcap_dump_close(des_pcap_dumper_t);
	}
	exit(0);
}
 
 
int main(int argc, char *argv[])
{
 
	//打开要处理pcap文件
	char errbuf[PCAP_ERRBUF_SIZE]={0};
	if( NULL==(source_pcap_t=pcap_open_offline(argv[1], errbuf)) )
	{
		printf("pcap_open_offline() return NULL.\nerrbuf:%s\n", errbuf);
		exit_main();
	}
	//打开保存的pcap文件	
	if( NULL==(des_pcap_dumper_t=pcap_dump_open(source_pcap_t,"./rescult.pcap")) )
	{
		printf("pcap_dump_open() fail.\n");
		exit_main();		
	}
 
	//判断链路层类型
	int datalink_offset=0;
	//pcap_datalink() returns the link-layer header type for the live capture or ``savefile'' specified by pcap_t
	int datalink_type=pcap_datalink(source_pcap_t);
	switch (datalink_type)
	{
		case PCAP_ERROR_NOT_ACTIVATED:
			printf("pcap_datalink() return PCAP_ERROR_NOT_ACTIVATED.\n");
			exit_main();
			break;
		case DLT_NULL:
			printf("Data Link Type:BSD loopback.\n");
			datalink_offset=4;
			break;
		case DLT_EN10MB:
			printf("Data Link Type:IEEE 802.3.\n");
			datalink_offset=14;
			break;
		case DLT_PPP:
			printf("Data Link Type:PPP.\n");
			datalink_offset=0;
			break;
		case DLT_FDDI:
			printf("Data Link Type:FDDI.\n");
			datalink_offset=21;
			break;
		case DLT_IEEE802_11:
			printf("Data Link Type:IEEE 802.11.\n");
			datalink_offset=22;
			break;
		case DLT_PPP_ETHER:
			printf("Data Link Type:PPPoE.\n");
			datalink_offset=20;
			break;
		default :
			printf("Data Link Type:UnKnown.\n");											
	}
	printf("datalink_offset:%d\n", datalink_offset);
 
	//设置过滤表达式
	struct bpf_program filter;
	if( -1==pcap_compile(source_pcap_t, &filter, "udp port 53", 1, 0) )
	{
		printf("pcap_compile() fail.\n");
		printf("errno:%s\n", pcap_geterr(source_pcap_t));
		exit_main();
	}
  	if( -1==pcap_setfilter(source_pcap_t, &filter) )
	{
		printf("pcap_setfilter() fail.\n");
		exit_main();
	}
 
	//网络层
	struct pcap_pkthdr packet;
	const u_char *pktStr;
	while(1)
	{
		pktStr=pcap_next(source_pcap_t, &packet);
		if( NULL==pktStr )
		{
			printf("pcap_next() return NULL.\n");
			break;		
		}
		else
		{
			printf("Packet length: %d\n", packet.len);  
	  		printf("Number of bytes: %d\n", packet.caplen);  
	 		printf("Recieved time: %s\n", ctime((const time_t *)&packet.ts.tv_sec));
			//读到的数据包写入生成pcap文件
			pcap_dump((u_char*)des_pcap_dumper_t, &packet, pktStr);	
		}
	}
	
	pcap_dump_close(des_pcap_dumper_t);
	pcap_close(source_pcap_t);
	return 0;
}
