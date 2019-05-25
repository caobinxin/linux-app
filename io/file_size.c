#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <malloc.h>
#include <stdio.h>

void main(int argc, char*argv[]){

    off_t file_size;
    char *buffer;
    struct stat stbuf;
    int fd;
      
    fd = open(argv[1], O_RDONLY);
    if (fd == -1) {
      /* Handle error */
    }
      
    if ((fstat(fd, &stbuf) != 0) || (!S_ISREG(stbuf.st_mode))) {
      /* Handle error */
    }
      
    file_size = stbuf.st_size;
    printf("file_size is %d\n", file_size);

    char buffer_size[100];
    sprintf(buffer_size, "%lld\n", file_size);
    printf("字符串输出大小: %s", buffer_size);
 
    // system("ls -l"); //直接可以在这里执行 shell 命令
    // printf("kkkkkkkkkkkkkkkkkkkkkkkk\n") ;
    return 0;
}




// struct stat {
//     dev_t     st_dev;     /* ID of device containing file */
//     ino_t     st_ino;     /* inode number */
//     mode_t    st_mode;    /* protection */
//     nlink_t   st_nlink;   /* number of hard links */
//     uid_t     st_uid;     /* user ID of owner */
//     gid_t     st_gid;     /* group ID of owner */
//     dev_t     st_rdev;    /* device ID (if special file) */
//     off_t     st_size;    /* total size, in bytes */
//     blksize_t st_blksize; /* blocksize for file system I/O */
//     blkcnt_t  st_blocks;  /* number of 512B blocks allocated */
//     time_t    st_atime;   /* time of last access */
//     time_t    st_mtime;   /* time of last modification */
//     time_t    st_ctime;   /* time of last status change */
// };

