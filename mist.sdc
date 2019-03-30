## Generated SDC file "constraints.sdc"

## Copyright (C) 1991-2011 Altera Corporation
## Your use of Altera Corporation's design tools, logic functions
## and other software and tools, and its AMPP partner logic
## functions, and any output files from any of the foregoing
## (including device programming or simulation files), and any
## associated documentation or information are expressly subject
## to the terms and conditions of the Altera Program License
## Subscription Agreement, Altera MegaCore Function License
## Agreement, or other applicable license agreement, including,
## without limitation, that your use is for the sole purpose of
## programming logic devices manufactured by Altera and sold by
## Altera or its authorized distributors. Please refer to the
## applicable agreement for further details.


## VENDOR "Altera"
## PROGRAM "Quartus II"
## VERSION "Version 11.1 Build 216 11/23/2011 Service Pack 1 SJ Web Edition"

## DATE "Fri Jul 06 23:05:47 2012"

##
## DEVICE "EP3C25E144C8"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name {clk_27} -period 37.037 -waveform { 0.000 18.500 } [get_ports {CLOCK_27[0]}]
create_clock -name {SPI_SCK}  -period 41.666 -waveform { 20.8 41.666 } [get_ports {SPI_SCK}]

#**************************************************************
# Create Generated Clock
#**************************************************************

derive_pll_clocks
create_generated_clock -name clk_8 -source [get_pins {clock|altpll_component|auto_generated|pll1|clk[1]}] -divide_by 4 [get_registers {clk_counter[1]}]
#**************************************************************
# Set Clock Latency
#**************************************************************


#**************************************************************
# Set Clock Uncertainty
#**************************************************************

derive_clock_uncertainty;

#**************************************************************
# Set Input Delay
#**************************************************************

set_input_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -max 6.4 [get_ports SDRAM_DQ[*]]
set_input_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -min 3.2 [get_ports SDRAM_DQ[*]]

#**************************************************************
# Set Output Delay
#**************************************************************

set_output_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -max 1.5 [get_ports {SDRAM_D* SDRAM_A* SDRAM_BA* SDRAM_n* SDRAM_CKE}]
set_output_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -min -0.8 [get_ports {SDRAM_D* SDRAM_A* SDRAM_BA* SDRAM_n* SDRAM_CKE}]
set_output_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[3]}] -max 1.5 [get_ports SDRAM_CLK]
set_output_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[3]}] -min -0.8 [get_ports SDRAM_CLK]

#**************************************************************
# Set Clock Groups
#**************************************************************

set_clock_groups -asynchronous -group [get_clocks {SPI_SCK}] -group [get_clocks {clock|altpll_component|auto_generated|pll1|clk[*]}]
set_clock_groups -asynchronous -group [get_clocks {SPI_SCK}] -group [get_clocks {clk_8}]
set_clock_groups -asynchronous -group [get_clocks {pll_mfp1|altpll_component|auto_generated|pll1|clk[0]}] -group [get_clocks {clock|altpll_component|auto_generated|pll1|clk[*]}]

#**************************************************************
# Set False Path
#**************************************************************

# Don't care about the debug overlay
set_false_path -to {video:video|overlay:overlay|*}
set_false_path -to [get_ports {VGA_*}]
set_false_path -to [get_ports {UART_TX}]
set_false_path -to [get_ports {AUDIO_L}]
set_false_path -to [get_ports {AUDIO_R}]
set_false_path -to [get_ports {LED}]

#**************************************************************
# Set Multicycle Path
#**************************************************************

# SDRAM 128 MHz to CPU 32 MHz -> 4x
set_multicycle_path -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[1]}] -setup 4
set_multicycle_path -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[1]}] -hold 3

# CPU -> 2x
set_multicycle_path -from {TG68KdotC_Kernel:tg68k|*} -setup 2
set_multicycle_path -from {TG68KdotC_Kernel:tg68k|*} -hold 1

# clk8 -> SDRAM 128MHz -> 4x
set_multicycle_path -from [get_clocks {clk_8}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -setup 4
set_multicycle_path -from [get_clocks {clk_8}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -hold 3

# CPU 32 to clk_8 -> 2x
set_multicycle_path -to [get_clocks {clk_8}] -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[1]}] -setup 2
set_multicycle_path -to [get_clocks {clk_8}] -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[1]}] -hold 1

# FX68K
set_multicycle_path -start -setup -from [get_keepers fx68k:fx68k|Ir[*]] -to [get_keepers fx68k:fx68k|microAddr[*]] 2
set_multicycle_path -start -hold -from [get_keepers fx68k:fx68k|Ir[*]] -to [get_keepers fx68k:fx68k|microAddr[*]] 1
set_multicycle_path -start -setup -from [get_keepers fx68k:fx68k|Ir[*]] -to [get_keepers fx68k:fx68k|nanoAddr[*]] 2
set_multicycle_path -start -hold -from [get_keepers fx68k:fx68k|Ir[*]] -to [get_keepers fx68k:fx68k|nanoAddr[*]] 1

#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************
