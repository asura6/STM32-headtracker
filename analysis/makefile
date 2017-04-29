TARGET = HID

CC = gcc

WRAP = $(TARGET)_wrap

IFC = $(TARGET).i
SRC = $(wildcard ./*.c) 
SRC += $(WRAP).c
OBJ = $(SRC:.c=.o)

default: _$(TARGET).so

$(OBJ).c: 
	swig -python $(IFC) 

$(WRAP).o: $(SRC)
	$(CC) -fpic -c $(SRC) -I/usr/include/python3.6m

%.so: $(OBJ)
	$(CC) -shared $(OBJ) -o _$(TARGET).so

clean: 
	rm -f $(WRAP).c ./*.o ./*.so $(TARGET).py 
