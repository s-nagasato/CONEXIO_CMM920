#CC = gcc #selfbuild
CC=arm-linux-gnueabi-gcc #crossbuild
CFLAGS= -g -Wall
VERSION=1.0.1

#INSTALL_DIR=/usr/local/lib/
INSTALL_DIR=/usr/arm-linux-gnueabi/lib/

all:libconexio_CMM920.o libconexio_CMM920_wrap.o libconexio_CMM920.so

libconexio_CMM920.o:	
	${CC} libconexio_CMM920_func.c -c -fPIC -o libconexio_CMM920.o
libconexio_CMM920_wrap.o:
	${CC} libconexio_CMM920_wrap.c -c -fPIC -o libconexio_CMM920_wrap.o	
libconexio_CMM920.so:
	${CC} -shared -O2 -Wl,-soname,libconexio_CMM920.so -o libconexio_CMM920.so.${VERSION} libconexio_CMM920.o libconexio_CMM920_wrap.o

install:
	cp libconexio_CMM920.so.${VERSION} ${INSTALL_DIR}
	ln -s ${INSTALL_DIR}libconexio_CMM920.so.${VERSION} ${INSTALL_DIR}libconexio_CMM920.so
	ldconfig

uninstall:
#	rm ${INSTALL_DIR}/libconexio_CMM920.so.*
	rm ${INSTALL_DIR}/libconexio_CMM920.*
	ldconfig

clean:
	rm -f *.o *.so *.so.*
