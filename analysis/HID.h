#include <stdint.h>

int16_t buf[7]; 

int Connect(const char device[]);
void Disconnect(int fd); 
int16_t* Read(int fd); 
