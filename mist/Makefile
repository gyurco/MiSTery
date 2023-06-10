QUARTUS_DIR=$(HOME)/altera/13.1/quartus/bin
### programs ###
MAP=$(QUARTUS_DIR)/quartus_map
FIT=$(QUARTUS_DIR)/quartus_fit
ASM=$(QUARTUS_DIR)/quartus_asm
PGM=$(QUARTUS_DIR)/quartus_pgm

### project ###
PROJECT=`pwd`/mist

TODAY = `date +"%m/%d/%y"`

### build rules ###

# all
all:
	@echo Making FPGA programming files ...
	@make map
	@make fit
	@make asm

map:
	@echo Running mapper ...
	@$(MAP) $(PROJECT)

fit:
	@echo Running fitter ...
	@$(FIT) $(PROJECT)

asm:
	@echo Running assembler ...
	@$(ASM) $(PROJECT)

# clean
clean:
	@echo clean
	@rm -rf ./out/
	@rm -rf ./db/
	@rm -rf ./incremental_db/
