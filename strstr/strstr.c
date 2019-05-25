#include <string.h>
#include <stdio.h>
 
int main()
{
    char *haystack = "Hello, this is csdn blog, I am here";
    char *needle = "is";
    char *temp;
 
    temp = strstr(haystack, needle);
    if(temp != NULL)
    {
        printf("%s\n", temp);
    }
    else
    {
        printf("can not find [%s] from [%s]\n", needle, haystack);
    }
 
    return 0;
}