%module HID 
%{
#include "HID.h" 
%} 

%include "carrays.i"
%include "stdint.i"

%array_class(int16_t, intArray); 

int Connect(const char device[]);
void Disconnect(int fd);
int16_t *Read(int fd); 
