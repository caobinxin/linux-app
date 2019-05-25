#include <stdio.h>
#include <string.h>

#define R_ENV "OLDPWD"
int main(int argc, const char *argv[])
{
	extern char **environ ;
	
	printf("environ = %s\n",*environ) ;
	printf("environ addr = %p\n", environ) ;

	char** env = environ;

	while( *env != NULL)
	{
		printf("%s\n", *env++) ;
	}

	/*读指定的 环境变量的值*/
	printf("**********************************************\n") ;
	printf("**********************************************\n") ;
	printf("**********************************************\n") ;
	printf("**********************************************\n") ;

	char* value = NULL ;
	env = environ ;
	while( *env != NULL)
	{
		char* str = *env++ ;
		if( strncmp(str , R_ENV, strlen(R_ENV)) == 0)
		{
			value = str ;
			break ;
		}
	}

	if( NULL != value)
	{
		printf("%s\n", value) ;
	
	}else{
		printf("null ===\n") ;
	}

	return 0;
}
