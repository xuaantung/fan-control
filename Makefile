# default target when "make" is run w/o arguments

all: main.rom

%.o: %.c
	avr-gcc -c -g -O3 -Wall -mmcu=atmega2560 -I. $*.c -o $*.o

%.elf: %.o
	avr-gcc $*.o -Wl,-Map=$*.map,--cref -mmcu=atmega2560 -o $*.elf

# copy ROM (FLASH) object
%.rom: %.elf
	avr-objcopy -O ihex $*.elf $*.rom

# command to clean up junk (no source files) (invoked by "make clean")
clean:
	rm -f *.o *.rom *.elf *.map *~

%-install:%.rom 
	avrdude -pm2560 -cstk600 -Pusb -V -Uflash:w:$*.rom
#	avrdude -pm2560 -catmelice -Pusb -Uflash:w:$*.rom
