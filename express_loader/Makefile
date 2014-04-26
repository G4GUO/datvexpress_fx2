INCLUDES = -I../include
CC = sdcc -mmcs51
BASENAME   = express_loader
BASENAME8  = express_loader8
BASENAME16 = express_loader16

$(BASENAME8).ihx: $(BASENAME).c
	$(CC) $(INCLUDES) -DUSB8 $(BASENAME).c  -o$(BASENAME8).ihx
	$(CC) $(INCLUDES) -DUSB16 $(BASENAME).c -o$(BASENAME16).ihx

clean:
	rm -f $(BASENAME8).asm
	rm -f $(BASENAME8).ihx
	rm -f $(BASENAME8).lnk
	rm -f $(BASENAME8).lst
	rm -f $(BASENAME8).map
	rm -f $(BASENAME8).mem
	rm -f $(BASENAME8).rel
	rm -f $(BASENAME8).rst
	rm -f $(BASENAME8).sym
	rm -f $(BASENAME16).asm
	rm -f $(BASENAME16).ihx
	rm -f $(BASENAME16).lnk
	rm -f $(BASENAME16).lst
	rm -f $(BASENAME16).map
	rm -f $(BASENAME16).mem
	rm -f $(BASENAME16).rel
	rm -f $(BASENAME16).rst
	rm -f $(BASENAME16).sym


