#include <fcntl.h>
#include <unistd.h> 
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h> 

int16_t buf[7]; 

int Connect(const char device[]) {
    int fd, res; 
    fd = open(device, O_RDWR);

    if (fd < 0) {
        perror("Unable to open device");
        return -1;
    } 
    return fd;
} 

void Disconnect(int fd) {
    close(fd);
}

int16_t *Read(int fd) { 
    read(fd, buf, 14);
    return buf;
} 
