#create WinUSB gadget linux device

all:wug

incl= -I/usr/local/oecore-x86_64/sysroots/cortexa7t2hf-neon-oe-linux-gnueabi/usr/include

#prepro=-D major=0 -Dminor=1 -Dmicro=1

wug: 
	echo pwd:$(PWD)
	echo build with debug info
	$(CC) $(incl) -lpthread -g wugadget.c -o winusbgadget

rc: 
	echo build release candidate
	$(CC) $(incl) -lpthread wugadget.c -o winusbgadget

clean:
	rm -f wugadget.o winusbgadget

help:
	echo cmds: all ; wug ; clean ; rc

#eof
