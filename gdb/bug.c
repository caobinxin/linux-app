#include <stdio.h>

int main(int argc, const char *argv[])
{
	char input , a, b, c;
	int i ;

	if( argc < 2)
	{
		printf("insufficient parameter!\n") ;
		return 1 ;
	}

	input = argv[1][0] ;

//	for( i = 0; i < 2; i++)
//	{
//		printf("Please input a character:\n") ;
//		scanf("%c", &input) ;

		switch(input){
			case 'a':
				printf("You input the character 'a'\n") ;
				break ;
			case 'b':
				printf("You input the character 'b'\n") ;
				break ;
			case 'c':
				printf("You input the character 'c'\n") ;
				break ;
			case 'd':
				printf("You input the character 'd'\n") ;
				break ;
			default:
				printf("What you input is not belong to the judgement of the program\n") ;
				break ;
		}
//	}
//	scanf("%c%c%c",&a, &b, &c) ;
//	printf("%c%c%c", a, b, c) ;
	return 0;
}
