#include"stdio.h"
 
#include"unistd.h"
 
#include "string.h"
 
#define fileNAME1 "test"
 
#define fileNAME2 "./liang"
 
 
int main(void)
 
{
 
    char name[BUFSIZ];//文件名字
    int flag = 1;//退出标志,0 exit
    printf("\t\t\t程序开始\n");
    printf("请输入要检查的文件(可包含路径,EOF退出):");
    scanf("%s",name);
 
    if( strcmp(name,"EOF") == 0 ){
        flag = 0;
    }
 
    while(flag){
 
        if(access(name,F_OK)==0){
 
            printf("文件存在\n");
 
            if(access(name,R_OK|W_OK)==0){
                printf("文件可读可写\n");
            }else
                printf("文件不可读或不可写\n");
 
 
            if(access(name,X_OK)==0){
            printf("文件可执行\n");
            }else
                printf("文件不可执行\n");
 
        }else
            printf("文件不存\n");
 
        printf("\n请输入要检查的文件(可包含路径,EOF退出):");
        scanf("%s",name);
        if( strcmp(name,"EOF") == 0 ){
            flag = 0;
 
        }
 
    }//while
 
    printf("\t\t\t程序结束\n");
    return 0;
}