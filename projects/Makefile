

DIRS=uart_echo blinky mpi_mcu #boot_loader cm_mcu #i2c-sensors 

DIRSCLEAN=$(addsuffix .clean,$(DIRS))

all:  $(DIRS)


$(DIRS):
	@$(MAKE) -C $@

clean: $(DIRSCLEAN)


$(DIRSCLEAN): %.clean:
	@$(MAKE) -C $* clean


.PHONY: all clean $(DIRS) $(DIRSCLEAN)
