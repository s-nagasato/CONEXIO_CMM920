CC = gcc #selfbuild
#CC=arm-linux-gnueabi-gcc #crossbuild
VERSION=1.0.0

all:conexio_CMM920_send.o conexio_CMM920_recv.o

conexio_CMM920_send.o:	
	${CC} -o CMM920SendTest_${VERSION} conexio_CMM920_send.c -lconexio_CMM920 -lSerialFunc

conexio_CMM920_recv.o:	
	${CC} -o CMM920RecvTest_${VERSION} conexio_CMM920_recv.c -lconexio_CMM920 -lSerialFunc


clean:
	rm -f *.o libconexio_CMM920_Test
