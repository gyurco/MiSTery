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

create_clock -name {clk_27} -period 37.037 -waveform { 0.000 18.500 } [get_ports {CLOCK_27}]
create_clock -name {SPI_SCK}  -period 41.666 -waveform { 20.8 41.666 } [get_ports {SPI_SCK}]

#**************************************************************
# Create Generated Clock
#**************************************************************

derive_pll_clocks

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

set_input_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -reference_pin [get_ports SDRAM_CLK] -max 6.4 [get_ports SDRAM_DQ[*]]
set_input_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -reference_pin [get_ports SDRAM_CLK] -min 3.2 [get_ports SDRAM_DQ[*]]

#**************************************************************
# Set Output Delay
#**************************************************************

set_output_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -reference_pin [get_ports SDRAM_CLK] -max 1.5 [get_ports {SDRAM_D* SDRAM_A* SDRAM_BA* SDRAM_n* SDRAM_CKE}]
set_output_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -reference_pin [get_ports SDRAM_CLK] -min -0.8 [get_ports {SDRAM_D* SDRAM_A* SDRAM_BA* SDRAM_n* SDRAM_CKE}]

set_output_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[2]}] -max 0 [get_ports {VGA_*}]
set_output_delay -clock [get_clocks {clock|altpll_component|auto_generated|pll1|clk[2]}] -min -5 [get_ports {VGA_*}]

#**************************************************************
# Set Clock Groups
#**************************************************************

set_clock_groups -asynchronous -group [get_clocks {SPI_SCK}] -group [get_clocks {clock|altpll_component|auto_generated|pll1|clk[*]}]
set_clock_groups -asynchronous -group [get_clocks {pll_mfp1|altpll_component|auto_generated|pll1|clk[0]}] -group [get_clocks {clock|altpll_component|auto_generated|pll1|clk[*]}]

#**************************************************************
# Set False Path
#**************************************************************

set_false_path -to [get_ports {SDRAM_CLK}]
set_false_path -to [get_ports {UART_TX}]
set_false_path -to [get_ports {AUDIO_L}]
set_false_path -to [get_ports {AUDIO_R}]
set_false_path -to [get_ports {LED}]

#**************************************************************
# Set Multicycle Path
#**************************************************************

# SDRAM 96 MHz to system 32 MHz
set_multicycle_path -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[1]}] -start -setup 2
set_multicycle_path -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[1]}] -start -hold 1

# System 32 MHz to SDRAM 96 MHz
set_multicycle_path -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[1]}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -end -setup 2
set_multicycle_path -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[1]}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -end -hold 1

# FX68K
set_multicycle_path -start -setup -from [get_keepers atarist_sdram:atarist|fx68k:fx68k|Ir[*]] -to [get_keepers atarist_sdram:atarist|fx68k:fx68k|microAddr[*]] 2
set_multicycle_path -start -hold -from [get_keepers atarist_sdram:atarist|fx68k:fx68k|Ir[*]] -to [get_keepers atarist_sdram:atarist|fx68k:fx68k|microAddr[*]] 1
set_multicycle_path -start -setup -from [get_keepers atarist_sdram:atarist|fx68k:fx68k|Ir[*]] -to [get_keepers atarist_sdram:atarist|fx68k:fx68k|nanoAddr[*]] 2
set_multicycle_path -start -hold -from [get_keepers atarist_sdram:atarist|fx68k:fx68k|Ir[*]] -to [get_keepers atarist_sdram:atarist|fx68k:fx68k|nanoAddr[*]] 1

# Viking 128 MHz to SDRAM 96 MHz
set_multicycle_path -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[2]}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -setup 2
set_multicycle_path -from [get_clocks {clock|altpll_component|auto_generated|pll1|clk[2]}] -to [get_clocks {clock|altpll_component|auto_generated|pll1|clk[0]}] -hold 1

set_multicycle_path -from {atarist_sdram:atarist|sdram:sdram|dout[*]} -to {atarist_sdram:atarist|sdram:sdram|*} -setup 2
set_multicycle_path -from {atarist_sdram:atarist|sdram:sdram|dout[*]} -to {atarist_sdram:atarist|sdram:sdram|*} -hold 1

# VGA
set_multicycle_path -start -to [get_ports {VGA_*}] -setup 4
set_multicycle_path -start -to [get_ports {VGA_*}] -hold 3

#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************
