CC           = gcc
CFLAGS       = -Wall -Os -Wl,-Map,test.map -D BUILD_TEST=1 -g
OBJCOPY      = objcopy

.SILENT:
.PHONY:  clean

all: test_aes test_bootloader

test_aes : aes.o test_aes.o
	$(CC) $(CFLAGS) aes.o test_aes.o -o test_aes

test_aes.o : test_aes.c
	# compiling test.c
	$(CC) $(CFLAGS) -c test_aes.c -o test_aes.o

test_bootloader: aes.o bootloader.o test_bootloader.o
	$(CC) $(CFLAGS) aes.o bootloader.o test_bootloader.o -o test_bootloader

test_bootloader.o : test_bootloader.c
	$(CC) $(CFLAGS) -c test_bootloader.c -o test_bootloader.o

bootloader.o : ../src/bootloader.h ../src/bootloader.c
	$(CC) $(CFLAGS) --include test_bootloader.h -c ../src/bootloader.c -o bootloader.o

aes.o : ../src/aes.h ../src/aes.c
	# compiling aes.c
	$(CC) $(CFLAGS) -c ../src/aes.c -o aes.o

clean:
	rm -f *.OBJ *.LST *.o *.gch *.out *.hex *.map
