#include <string.h>
#include <stdio.h>
#include <unistd.h>

void test_string(){
    char *str = "cmd_upload";
    char str2[] = "cao" ;
    int len = strlen(str) ;
     strncmp("", str, strlen(str));
     printf("str = %s strlen = %d\n", str, len ) ;

     strcat(str2, "bin") ;
     printf("%s\n", str2) ;
}

void test_sh_return_value()
{
    execlp("bash", "bash", "./test.sh", (char *)NULL) ;
}
int
main(int argc, char const *argv[])
{
    test_sh_return_value() ;
    return 0;
}
