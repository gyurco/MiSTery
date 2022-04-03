/********************************************/
/*       Atari ST/STe/Mega STe core         */
/********************************************/

module atarist_sdram (
	// System clocks / reset / settings
	input wire           clk_96,
	input wire           clk_32,
	input wire           clk_128,
	input wire           clk_2,
	input wire           clk_mfp,
	input wire           porb,
	input wire    [31:0] system_ctrl,

	// Video output
	output wire    [3:0] r,
	output wire    [3:0] g,
	output wire    [3:0] b,
	output wire          hsync_n,
	output wire          vsync_n,
	output reg           monomode,
	output wire          blank_n,

	output reg           viking_active,
	output wire    [3:0] viking_r,
	output wire    [3:0] viking_g,
	output wire    [3:0] viking_b,
	output wire          viking_hs,
	output wire          viking_vs,

	// Sound output
	output wire    [14:0] audio_mix_l,
	output wire    [14:0] audio_mix_r,

	// MIDI OUT (parallel data)
	output wire          midi_out_strobe,
	output wire [7:0]    midi_out,

	// MIDI UART
	input wire           midi_rx,
	output wire          midi_tx,

	// Parallel port OUT
	input  wire          parallel_in_strobe,
	input  wire    [7:0] parallel_in,
	output wire          parallel_out_strobe,
	output wire    [7:0] parallel_out,
	input  wire          parallel_printer_busy,

	// Serial port IN/OUT
	// serial rs232 connection to io controller
	output               serial_data_out_available,
	input                serial_strobe_out,
	output         [7:0] serial_data_out,
	output        [63:0] serial_status_out,

	// serial rs223 connection from io controller
	input                serial_strobe_in,
	input          [7:0] serial_data_in,

	// ROM/CART download / ACSI
	input wire           data_in_strobe_rom,
	input wire           data_in_strobe_acsi,
	input wire    [15:0] data_in_reg,
	input wire    [23:1] data_addr,
	input wire           data_download,

	input wire           data_out_strobe,
	output wire   [15:0] data_out_reg,
	input wire           dma_ack,
	input wire     [7:0] dma_status,
	input wire           dma_nak,
	output wire    [7:0] dma_status_in,
	input wire     [3:0] dma_status_index,

	// FDC
	input wire       [1:0] img_mounted, // signaling that new image has been mounted
	input wire       [1:0] img_wp,      // write protect
	input wire      [31:0] img_size,    // size of image in bytes
	output reg      [31:0] sd_lba,
	output reg       [1:0] sd_rd,
	output reg       [1:0] sd_wr,
	input wire             sd_ack,
	input wire       [8:0] sd_buff_addr,
	input wire       [7:0] sd_dout,
	output wire      [7:0] sd_din,
	input wire             sd_dout_strobe,

	output wire            LED,

	// Ethernet interface
	output          [31:0] eth_status,

	input                  eth_mac_begin,
	input                  eth_mac_strobe,
	input            [7:0] eth_mac_byte,

	input                  eth_tx_read_begin,
	input                  eth_tx_read_strobe,
	output           [7:0] eth_tx_read_byte,

	input                  eth_rx_write_begin,
	input                  eth_rx_write_strobe,
	input            [7:0] eth_rx_write_byte,

	// PS2 keyboard data
	input wire             ps2_kbd_clk,
	input wire             ps2_kbd_data,

	// PS2 mouse data
	input wire             ps2_mouse_clk,
	input wire             ps2_mouse_data,

	// joysticks
	input wire     [15:0] joy0,
	input wire     [15:0] joy1,

	// RTC
	input wire     [63:0] rtc,

	// SDRAM
	inout wire  [ 16-1:0] SDRAM_DQ,   // SDRAM Data bus 16 Bits
	output wire [ 13-1:0] SDRAM_A,    // SDRAM Address bus 13 Bits
	output wire           SDRAM_DQML, // SDRAM Low-byte Data Mask
	output wire           SDRAM_DQMH, // SDRAM High-byte Data Mask
	output wire           SDRAM_nWE,  // SDRAM Write Enable
	output wire           SDRAM_nCAS, // SDRAM Column Address Strobe
	output wire           SDRAM_nRAS, // SDRAM Row Address Strobe
	output wire           SDRAM_nCS,  // SDRAM Chip Select
	output wire  [ 2-1:0] SDRAM_BA    // SDRAM Bank Address
);

parameter TG68K_ENABLE = 1'b0;

wire init = ~porb;

wire [1:0] cpu = system_ctrl[5:4];

// enable additional ste/megaste features
wire ste = system_ctrl[23] || system_ctrl[24];
wire mste = system_ctrl[24];
wire steroids = system_ctrl[23] && system_ctrl[24];  // a STE on steroids

// ethernec is enabled by the io controller whenever a USB 
// ethernet interface is detected
wire       ethernec_present = system_ctrl[25];
wire       cubase_enable = system_ctrl[30];

wire       psg_stereo = system_ctrl[22];

// STe always has a blitter
wire       blitter_en = (system_ctrl[19] || ste);
wire       viking_en = system_ctrl[28];
wire [8:0] acsi_enable = system_ctrl[17:10];
wire       mono_monitor = system_ctrl[8];

// RAM size selects
wire MEM512K = (system_ctrl[3:1] == 3'd0);
wire MEM1M   = (system_ctrl[3:1] == 3'd1);
wire MEM2M   = (system_ctrl[3:1] == 3'd2);
wire MEM4M   = (system_ctrl[3:1] == 3'd3);
wire MEM8M   = (system_ctrl[3:1] == 3'd4);
wire MEM14M  = (system_ctrl[3:1] == 3'd5);

// registered reset signals
reg         reset;
reg         peripheral_reset;
reg         ikbd_reset;
reg         mcu_reset_n = 1'b1;
reg   [6:0] reset_cnt = 7'h7f;
reg         cpu_reset_n_d;
wire        ext_reset = system_ctrl[0];

always @(posedge clk_32) begin

	cpu_reset_n_d <= cpu_reset_n_o;

	reset <= reset_cnt != 0;

	if (reset_cnt != 0) reset_cnt <= reset_cnt - 1'd1;
	if (ext_reset) reset_cnt <= 7'h7f;

	peripheral_reset <= reset | ~cpu_reset_n_o;

	// don't keep the GSTMCU in reset, because its signals are needed for the SDRAM controller
	if (reset_cnt == 10) mcu_reset_n <= 0;
	else if (reset_cnt < 8) mcu_reset_n <= 1;
	if (~cpu_reset_n_o & cpu_reset_n_d) mcu_reset_n <= 0;
end

always @(posedge clk_2) ikbd_reset <= peripheral_reset;

// MCU signals

wire        mhz4, mhz4_en, clk16, clk16_en = ~clk16;
wire        mcu_dtack_n;
wire        rom0_n, rom1_n, rom2_n, rom3_n, rom4_n, rom5_n, rom6_n, romp_n;
wire        ras0_n, ras1_n;
wire        mfpint_n, mfpcs_n, mfpiack_n;
wire        sndir, sndcs;
wire        n6850, fcs_n;
wire        rtccs_n, rtcrd_n, rtcwr_n;
wire        sint;
wire [15:0] mcu_dout;
wire        mcu_oe_l, mcu_oe_h;
wire        ras_n = ras0_n & ras1_n;
wire        button_n, joywe_n, joyrl_n, joywl, joyrh_n;

// dma
wire        rdy_o, rdy_i, mcu_bg_n, mcu_br_n, mcu_bgack_n;

// compatibility for viking
wire  [1:0] bus_cycle;

// for other peripherals
wire        iodevice = ~as_n & fc2 & (fc0 ^ fc1) & mbus_a[23:16] == 8'hff;

// CPU signals
wire        mhz8, mhz8_en1, mhz8_en2;
wire        berr_n;
wire        ipl0_n, ipl1_n, ipl2_n;
wire        cpu_fc0, cpu_fc1, cpu_fc2;
wire        cpu_as_n, cpu_rw, cpu_uds_n, cpu_lds_n, vma_n, vpa_n, cpu_E;
wire        cpu_reset_n_o;
wire [15:0] cpu_din, cpu_dout;
wire [23:1] cpu_a;

wire        rom_n = rom0_n & rom1_n & rom2_n & ((rom3_n & rom4_n) | cubase_enable | eth_rd) & rom5_n & rom6_n & romp_n;
assign      cpu_din = 
              ((~fcs_n & rw) ? dma_data_out : 16'hffff) &
              (blitter_sel ? blitter_data_out : 16'hffff) &
              (rdat_n ? 16'hffff : shifter_dout) &
              {8'hff, (mfpcs_n & mfpiack_n) ? 8'hff : mfp_data_out} &
              {(!rom3_n & cubase_enable) ? cubase_dout : 8'hff, 8'hff} &
              {eth_rd ? eth_data_out : 8'hff, 8'hff} &
              (rom_n ? 16'hffff : rom_data_out) &
              {(n6850 & rw) ? (mbus_a[2] ? midi_acia_data_out : kbd_acia_data_out) : 8'hff, 8'hff} &
              {snd_data_oe_l ? 8'hff : snd_data_out, 8'hff} &
              {8'hff, (mste_ctrl_sel & rw) ? mste_ctrl_data_out : 8'hff} &
              {12'hfff, button_n ? 4'hf : ste_buttons} &
              {joyrh_n ? 8'hff : ste_joy_in[15:8], joyrl_n ? 8'hff : ste_joy_in[7:0]} &
              {12'hfff, (rtccs_n & rw) ? 4'hf : rtc_data_out} &
              {mcu_oe_h ? mcu_dout[15:8] : 8'hff, mcu_oe_l ? mcu_dout[7:0] : 8'hff};

// Shifter signals
wire        cmpcs_n, latch, de, rdat_n, wdat_n, dcyc_n, sreq, sload_n, mono;
wire [15:0] shifter_dout;
wire [ 7:0] dma_snd_l, dma_snd_r;

// RAM signals
wire [23:1] ram_a;
wire        ram_uds, ram_lds, ram_we_n, ram_ref;
wire [15:0] ram_din;

// combined bus signals
wire        fc0 = blitter_has_bus ? blitter_fc0 : cpu_fc0;
wire        fc1 = blitter_has_bus ? blitter_fc1 : cpu_fc1;
wire        fc2 = blitter_has_bus ? blitter_fc2 : cpu_fc2;
wire        as_n = blitter_has_bus ? blitter_as_n : cpu_as_n;
wire        rw = blitter_has_bus ? blitter_rw_n : cpu_rw;
wire        uds_n = blitter_has_bus ? blitter_ds_n : cpu_uds_n;
wire        lds_n = blitter_has_bus ? blitter_ds_n : cpu_lds_n;
wire [23:1] mbus_a = blitter_has_bus ? blitter_addr : cpu_a;
// dout from the current bus master - TODO: merge with cpu_din after adding output enables to GSTMCU
wire [15:0] mbus_dout = !rdat_n ? shifter_dout :
                        !rom_n   ? rom_data_out :
                        blitter_sel ? blitter_data_out :
                        ~rdy_i ? dma_data_out :
                        cpu_dout;

wire        dtack_n = mcu_dtack_n_adj & ~mfp_dtack & ~mste_ctrl_sel & ~vme_sel & blitter_dtack_n;

/* ------------------------------------------------------------------------------ */
/* ------------------------------ GSTMCU + Shifter ------------------------------ */
/* ------------------------------------------------------------------------------ */

gstmcu gstmcu (
	.clk32      ( clk_32 ),
	.resb       ( mcu_reset_n ),
	.porb       ( porb ),
	.FC0        ( fc0 ),
	.FC1        ( fc1 ),
	.FC2        ( fc2 ),
	.AS_N       ( as_n ),
	.RW         ( rw ),
	.UDS_N      ( uds_n ),
	.LDS_N      ( lds_n ),
	.VMA_N      ( vma_n ),
	.MFPINT_N   ( mfpint_n ),
	.A          ( mbus_a ), // from CPU bus
	.ADDR       ( ram_a ),  // to RAM
	// DIN - only interested in sources which can be bus masters (+shifter) - to avoid long combinatorial paths
	.DIN        ( ~rdy_i ? dma_data_out : blitter_sel ? blitter_data_out : !rdat_n  ? shifter_dout : cpu_dout ),
	.DOUT       ( mcu_dout ),
	.OE_L       ( mcu_oe_l ),
	.OE_H       ( mcu_oe_h ),
	.CLK_O      ( clk16 ),
	.MHZ8       ( mhz8 ),
	.MHZ8_EN1   ( mhz8_en1 ),
	.MHZ8_EN2   ( mhz8_en2 ),
	.MHZ4       ( mhz4 ),
	.MHZ4_EN    ( mhz4_en ),
	.RDY_N_I    ( rdy_o ),
	.RDY_N_O    ( rdy_i ),
	.BG_N       ( mcu_bg_n ),
	.BR_N_I     ( blitter_br_n ),
	.BR_N_O     ( mcu_br_n ),
	.BGACK_N_I  ( 1'b1 ),
	.BGACK_N_O  ( mcu_bgack_n ),
	.BERR_N     ( berr_n ),
	.IPL0_N     ( ipl0_n ),
	.IPL1_N     ( ipl1_n ),
	.IPL2_N     ( ipl2_n ),
	.DTACK_N_I  ( dtack_n ),
	.DTACK_N_O  ( mcu_dtack_n ),
	.IACK_N     ( mfpiack_n),
	.ROM0_N     ( rom0_n ),
	.ROM1_N     ( rom1_n ),
	.ROM2_N     ( rom2_n ),
	.ROM3_N     ( rom3_n ),
	.ROM4_N     ( rom4_n ),
	.ROM5_N     ( rom5_n ),
	.ROM6_N     ( rom6_n ),
	.ROMP_N     ( romp_n ),
	.RAM_N      ( ),
	.RAS0_N     ( ras0_n ),
	.RAS1_N     ( ras1_n ),
	.RAM_LDS    ( ram_lds ),
	.RAM_UDS    ( ram_uds ),
	.REF        ( ram_ref ),
	.VPA_N      ( vpa_n ),
	.MFPCS_N    ( mfpcs_n ),
	.SNDIR      ( sndir ),
	.SNDCS      ( sndcs ),
	.N6850      ( n6850 ),
	.FCS_N      ( fcs_n ),
	.RTCCS_N    ( rtccs_n ),
	.RTCRD_N    ( rtcrd_n ),
	.RTCWR_N    ( rtcwr_n ),
	.LATCH      ( latch ),
	.HSYNC_N    ( hsync_n ),
	.VSYNC_N    ( vsync_n ),
	.DE         ( de ),
	.BLANK_N    ( blank_n ),
	.RDAT_N     ( rdat_n ),
	.WE_N       ( ram_we_n ),
	.WDAT_N     ( wdat_n ),
	.CMPCS_N    ( cmpcs_n ),
	.DCYC_N     ( dcyc_n ),
	.SREQ       ( sreq),
	.SLOAD_N    ( sload_n),
	.SINT       ( sint ),

	.BUTTON_N   ( button_n ),
	.JOYWE_N    ( joywe_n  ),
	.JOYRL_N    ( joyrl_n  ),
	.JOYWL      ( joywl    ),
	.JOYRH_N    ( joyrh_n  ),

	.st            ( ~ste ),
	.extra_ram     ( MEM8M | MEM14M ),
	.tos192k       ( tos192k ),
	.turbo         ( turbo_bus ),
	.viking_at_c0  ( viking_enable && !steroids ),
	.viking_at_e8  ( viking_enable &&  steroids ),
	.bus_cycle     ( bus_cycle )
);

gstshifter gstshifter (
	.clk32      ( clk_32 ),
	.ste        ( ste ),
	// resb originally cmpcs_n | dcyc_n internally in shifter,
	// but we have the luxury of having a reset pin
	.resb       ( ~peripheral_reset ),

	// CPU/RAM interface
	.CS         ( ~cmpcs_n ),
	.A          ( mbus_a[6:1] ),
	.DIN        ( mbus_dout ),
	.DOUT       ( shifter_dout ),
	.LATCH      ( latch ),
	.RDAT_N     ( rdat_n ),   // latched MDIN -> DOUT
	.WDAT_N     ( wdat_n ),   // DIN  -> MDOUT
	.RW         ( rw ),
	.MDIN       ( ram_data_out ),
	.MDOUT      ( ram_din  ),

	// VIDEO
	.MONO_OUT   ( mono ),
	.LOAD_N     ( dcyc_n ),
	.DE         ( de ),
	.BLANK_N    ( blank_n ),
	.R          ( r ),
	.G          ( g ),
	.B          ( b ),

	// DMA SOUND
	.SLOAD_N    ( sload_n ),
	.SREQ       ( sreq ),
	.audio_left ( dma_snd_l ),
	.audio_right( dma_snd_r )
);

// --------------- the Viking compatible 1280x1024 graphics card -----------------

// viking/sm194 is enabled and max 8MB memory may be enabled. In steroids mode
// video memory is moved to $e80000 and all stram up to 14MB may be used
wire viking_mem_ok = MEM512K || MEM1M || MEM2M || MEM4M || MEM8M;
wire viking_enable = (viking_en && viking_mem_ok) || steroids;

// check for cpu access to 0xcxxxxx with viking enabled to switch video
// output once the driver loads. 256 accesses to the viking memory range
// are considered a valid sign that the driver is working. Without driver
// others may also probe that area which is why we want to see 256 accesses
reg [7:0] viking_in_use;

always @(posedge clk_32) begin
	if(reset) begin
		viking_in_use <= 8'h00;
		viking_active <= 1'b0;
	end else begin
		// cpu writes to $c0xxxx or $e80000
		if(mhz8_en1 && !as_n && viking_enable &&
		  (mbus_a[23:18] == (steroids?6'b111010:6'b110000)) && (viking_in_use != 8'hff))
			viking_in_use <= viking_in_use + 1'd1;

		viking_active <= (viking_in_use == 8'hff);
	end
end

wire [23:1] viking_vaddr;
wire viking_read;

viking viking (
	.pclk      ( clk_128         ), // 128MHz
	.himem     ( steroids        ),
	.clk_8_en  ( mhz8_en1        ), // 8 MHz bus clock
	.bus_cycle ( { viking_precycle, viking_cycle } ), // bus-cycle to sync video memory access with cpu

	// memory interface
	.addr      ( viking_vaddr    ), // video word address
	.read      ( viking_read     ), // video read cycle
	.data      ( ram_data_out64  ), // video data read

	// video output
	.hs        ( viking_hs       ),
	.vs        ( viking_vs       ),
	.r         ( viking_r        ),
	.g         ( viking_g        ),
	.b         ( viking_b        )
);

// assume mono mode only if it's set during VSYNC
// demos like to switch it on/off during active display to get rid of borders
always @(posedge clk_32) begin
	if (!vsync_n) monomode <= mono;
end

/* ------------------------------------------------------------------------------ */
/* ------------------------------------ CPU ------------------------------------- */
/* ------------------------------------------------------------------------------ */

reg         use_16mhz;
reg         turbo_bus;

always @(posedge clk_32)
	if (mhz8_en1 & as_n) begin
		use_16mhz <= (enable_16mhz | steroids);
		turbo_bus <= (enable_cache | steroids);
	end

wire        phi1 = use_16mhz ?  clk16_en : mhz8_en1;
wire        phi2 = use_16mhz ? ~clk16_en : mhz8_en2;

wire        shifter_cycle = (turbo_bus && (bus_cycle == 0 || bus_cycle == 3)) || (!turbo_bus && bus_cycle == 2);
wire        mcu_dtack_n_adj = (use_16mhz & ~rom_n) ? (mcu_dtack_n | shifter_cycle) : mcu_dtack_n;

wire        is68000 = cpu == 0 || !TG68K_ENABLE;
assign      cpu_rw        = is68000 ? fx68_rw : tg68_rw;
assign      cpu_as_n      = is68000 ? fx68_as_n : tg68_as_n;
assign      cpu_uds_n     = is68000 ? fx68_uds_n : tg68_uds_n;
assign      cpu_lds_n     = is68000 ? fx68_lds_n : tg68_lds_n;
assign      cpu_E         = is68000 ? fx68_E : tg68_E;
assign      vma_n         = is68000 ? fx68_vma_n : tg68_vma_n;
assign      cpu_fc0       = is68000 ? fx68_fc0 : tg68_fc0;
assign      cpu_fc1       = is68000 ? fx68_fc1 : tg68_fc1;
assign      cpu_fc2       = is68000 ? fx68_fc2 : tg68_fc2;
assign      cpu_a         = is68000 ? fx68_a : tg68_a[23:1];
assign      cpu_dout      = is68000 ? fx68_dout : tg68_dout;
assign      blitter_bg_n  = is68000 ? fx68_bg_n : tg68_bg_n;
assign      cpu_reset_n_o = is68000 ? fx68_reset_n : tg68_reset_n;

wire        fx68_rw;
wire        fx68_as_n;
wire        fx68_uds_n;
wire        fx68_lds_n;
wire        fx68_E;
wire        fx68_vma_n;
wire        fx68_fc0;
wire        fx68_fc1;
wire        fx68_fc2;
wire        fx68_bg_n;
wire        fx68_reset_n;
wire [15:0] fx68_dout;
wire [23:1] fx68_a;

fx68k fx68k (
	.clk        ( clk_32     ),
	.extReset   ( reset      ),
	.pwrUp      ( reset      ),
	.enPhi1     ( phi1       ),
	.enPhi2     ( phi2       ),

	.eRWn       ( fx68_rw    ),
	.ASn        ( fx68_as_n  ),
	.LDSn       ( fx68_lds_n ),
	.UDSn       ( fx68_uds_n ),
	.E          ( fx68_E     ),
	.VMAn       ( fx68_vma_n ),
	.FC0        ( fx68_fc0   ),
	.FC1        ( fx68_fc1   ),
	.FC2        ( fx68_fc2   ),
	.BGn        ( fx68_bg_n  ),
	.oRESETn    ( fx68_reset_n ),
	.oHALTEDn   (),
	.DTACKn     ( dtack_n    ),
	.VPAn       ( vpa_n      ),
	.BERRn      ( berr_n     ),
	.HALTn      ( 1'b1       ),
	.BRn        ( blitter_br_n & mcu_br_n ),
	.BGACKn     ( blitter_bgack_n ),
	.IPL0n      ( ipl0_n     ),
	.IPL1n      ( ipl1_n     ),
	.IPL2n      ( ipl2_n     ),
	.iEdb       ( cpu_din    ),
	.oEdb       ( fx68_dout  ),
	.eab        ( fx68_a     )
);

wire        tg68_rw;
wire        tg68_as_n;
wire        tg68_uds_n;
wire        tg68_lds_n;
wire        tg68_E;
wire        tg68_vma_n;
wire        tg68_fc0;
wire        tg68_fc1;
wire        tg68_fc2;
wire        tg68_bg_n;
wire        tg68_reset_n;
wire [15:0] tg68_dout;
wire [23:0] tg68_a;

tg68k tg68k (
	.clk       ( clk_32 ),
	.reset     ( reset  ),
	.phi1      ( phi1   ),
	.phi2      ( phi2   ),
	.cpu       ( 2'b11  ),

	.dtack_n   ( dtack_n ),
	.rw_n      ( tg68_rw ),
	.as_n      ( tg68_as_n ),
	.uds_n     ( tg68_uds_n ),
	.lds_n     ( tg68_lds_n ),
	.fc        ( { tg68_fc2, tg68_fc1, tg68_fc0 } ),
	.reset_n   ( tg68_reset_n ),

	.E         ( tg68_E ),
	.vma_n     ( tg68_vma_n ),
	.vpa_n     ( vpa_n ),

	.br_n      ( blitter_br_n & mcu_br_n ),
	.bg_n      ( tg68_bg_n ),
	.bgack_n   ( blitter_bgack_n ),

	.ipl       ( { ipl2_n, ipl1_n, ipl0_n } ),
	.berr      ( ~berr_n ),
	.din       ( cpu_din ),
	.dout      ( tg68_dout ),
	.addr      ( tg68_a )
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------------ MFP ------------------------------------- */
/* ------------------------------------------------------------------------------ */

wire acia_irq = kbd_acia_irq || midi_acia_irq;

// the STE delays the xsirq by 1/250000 second before feeding it into timer_a
// 74ls164
wire      xsint = ~sint;
reg [7:0] xsint_delay;
always @(posedge clk_32 or negedge xsint) begin
	if(!xsint) xsint_delay <= 8'h00;            // async reset
	else if (clk_2_en) xsint_delay <= {xsint_delay[6:0], xsint};
end

wire xsint_delayed = xsint_delay[7];

// mfp io7 is mono_detect which in ste is xor'd with the dma sound irq
wire mfp_io7 = mono_monitor ^ (ste?xsint:1'b0);

// inputs 1,2 and 6 are outputs from an MC1489 serial receiver
wire  [7:0] mfp_gpio_in = {mfp_io7, 1'b1, !(acsi_irq | fdc_irq), !acia_irq, blitter_irq_n, 2'b11, !parallel_printer_busy};
wire  [1:0] mfp_timer_in = {de, ste?xsint_delayed:!parallel_printer_busy};
wire  [7:0] mfp_data_out;
wire        mfp_dtack;

wire        mfp_int, mfp_iack = ~mfpiack_n;
assign      mfpint_n = ~mfp_int;

mfp mfp (
	// cpu register interface
	.clk      ( clk_32        ),
	.clk_en   ( mhz4_en       ),
	.reset    ( peripheral_reset ),
	.din      ( mbus_dout[7:0]),
	.sel      ( ~mfpcs_n      ),
	.addr     ( mbus_a[5:1]   ),
	.ds       ( lds_n         ),
	.rw       ( rw            ),
	.dout     ( mfp_data_out  ),
	.irq      ( mfp_int       ),
	.iack     ( mfp_iack      ),
	.dtack    ( mfp_dtack     ),

	// serial/rs232 interface io-controller<->mfp
	.serial_data_out_available (serial_data_out_available),
	.serial_strobe_out         (serial_strobe_out),
	.serial_data_out           (serial_data_out),
	.serial_status_out         (serial_status_out),

	.serial_strobe_in          (serial_strobe_in),
	.serial_data_in            (serial_data_in),

	// input signals
	.clk_ext  ( clk_mfp       ),  // 2.457MHz clock
	.t_i      ( mfp_timer_in  ),  // timer a/b inputs
	.i        ( mfp_gpio_in   )   // gpio-in
);

/* ------------------------------------------------------------------------------ */
/* ---------------------------------- IKBD -------------------------------------- */
/* ------------------------------------------------------------------------------ */

wire ikbd_tx, ikbd_rx;
wire joy_port_ste;

ikbd ikbd (
	.clk(clk_2),
	.res(ikbd_reset),

	.ps2_kbd_clk(ps2_kbd_clk),
	.ps2_kbd_data(ps2_kbd_data),
	.ps2_mouse_clk(ps2_mouse_clk),
	.ps2_mouse_data(ps2_mouse_data),
	.tx(ikbd_tx),
	.rx(ikbd_rx),
	.joystick0(joy_port_ste ? 5'd0 : {joy0[5:4], joy0[0], joy0[1], joy0[2], joy0[3]}),
	.joystick1(joy_port_ste ? 5'd0 : {joy1[  4], joy1[0], joy1[1], joy1[2], joy1[3]}),
	.joy_port_toggle(joy_port_ste)
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------- keyboard ACIA -------------------------------- */
/* ------------------------------------------------------------------------------ */

wire [7:0] kbd_acia_data_out;
wire       kbd_acia_irq;

acia kbd_acia (
	// cpu interface
	.clk      ( clk_32             ),
	.E        ( cpu_E              ),
	.reset    ( reset              ),
	.din      ( mbus_dout[15:8]    ),
	.sel      ( n6850 & ~mbus_a[2] ),
	.rs       ( mbus_a[1]          ),
	.rw       ( rw                 ),
	.dout     ( kbd_acia_data_out  ),
	.irq      ( kbd_acia_irq       ),

	.rx       ( ikbd_tx            ),
	.tx       ( ikbd_rx            )
);

/* ------------------------------------------------------------------------------ */
/* --------------------------------- MIDI ACIA ---------------------------------- */
/* ------------------------------------------------------------------------------ */

wire [7:0] midi_acia_data_out;
wire       midi_acia_irq;

assign     midi_out = mbus_dout[15:8];

acia midi_acia (
	// cpu interface
	.clk      ( clk_32             ),
	.E        ( cpu_E              ),
	.reset    ( reset              ),
	.din      ( mbus_dout[15:8]    ),
	.sel      ( n6850 & mbus_a[2]  ),
	.rs       ( mbus_a[1]          ),
	.rw       ( rw                 ),
	.dout     ( midi_acia_data_out ),
	.irq      ( midi_acia_irq      ),

	.rx       ( midi_rx            ),
	.tx       ( midi_tx            ),

	// redirected midi interface
	.dout_strobe ( midi_out_strobe )
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------------ PSG ------------------------------------- */
/* ------------------------------------------------------------------------------ */

wire [7:0] snd_data_out;
wire       snd_data_oe_l;
wire [7:0] ym_a_out, ym_b_out, ym_c_out;

wire [9:0] ym_audio_out_l;
wire [9:0] ym_audio_out_r;

reg clk_2_en;
always @(posedge clk_32) begin
	reg [3:0] cnt;
	clk_2_en <= (cnt == 0);
	cnt <= cnt + 1'd1;
end

wire [7:0] port_b_in = parallel_in;
wire [7:0] port_a_in = { port_a_out[7:6], parallel_in_strobe, port_a_out[4:0] };
wire [7:0] port_a_out;
wire [7:0] port_b_out;
wire       floppy_side = port_a_out[0];
wire [1:0] floppy_sel = port_a_out[2:1];

assign     parallel_out_strobe = port_a_out[5];
assign     parallel_out = port_b_out;

YM2149 #(.MIXER_VOLTABLE(1'b1)) ym2149 (
	.CLK         ( clk_32        ),
	.ENA         ( clk_2_en      ),
	.RESET_L     ( ~peripheral_reset ),
	.I_DA        ( mbus_dout[15:8]),
	.O_DA        ( snd_data_out  ),
	.O_DA_OE_L   ( snd_data_oe_l ),
	.O_AUDIO_L   ( ym_audio_out_l),
	.O_AUDIO_R   ( ym_audio_out_r),
	.I_BDIR      ( sndir         ),
	.I_BC1       ( sndcs         ),
	.I_STEREO    ( psg_stereo    ),
	.I_IOA       ( port_a_in     ),
	.O_IOA       ( port_a_out    ),
	.I_IOB       ( port_b_in     ),
	.O_IOB       ( port_b_out    )
);

// audio output processing

// YM and STE audio channels are expanded to 14 bits and added resulting in 15 bits
// for the sigmadelta dac take from the minimig

// This should later be handled by the lmc1992

wire [9:0] ym_audio_out_l_signed = ym_audio_out_l - 10'h200;
wire [9:0] ym_audio_out_r_signed = ym_audio_out_r - 10'h200;
wire [7:0] ste_audio_out_l_signed = dma_snd_l - 8'h80;
wire [7:0] ste_audio_out_r_signed = dma_snd_r - 8'h80;

assign audio_mix_l =
        { ym_audio_out_l_signed[9], ym_audio_out_l_signed, ym_audio_out_l_signed[9:6]} +
        { ste_audio_out_l_signed[7], ste_audio_out_l_signed, ste_audio_out_l_signed[7:2] };
assign audio_mix_r =
        { ym_audio_out_r_signed[9], ym_audio_out_r_signed, ym_audio_out_r_signed[9:6]} +
        { ste_audio_out_r_signed[7], ste_audio_out_r_signed, ste_audio_out_r_signed[7:2] };

/* ------------------------------------------------------------------------------ */
/* ------------------------------ Mega STe control ------------------------------ */
/* ------------------------------------------------------------------------------ */

// mega ste cache controller 8 bit interface at $ff8e20 - $ff8e21
// STEroids mode does not have this config, it always runs full throttle
wire       mste_ctrl_sel = !steroids && mste && iodevice && !lds_n && ({mbus_a[15:1], 1'd0} == 16'h8e20);
wire [7:0] mste_ctrl_data_out;
wire       enable_16mhz, enable_cache;

mste_ctrl mste_ctrl (
	// cpu register interface
	.clk      ( clk_32             ),
	.reset    ( reset              ),
	.din      ( mbus_dout[7:0]     ),
	.sel      ( mste_ctrl_sel      ),
	.rw       ( rw                 ),
	.dout     ( mste_ctrl_data_out ),

	.enable_cache ( enable_cache   ),
	.enable_16mhz ( enable_16mhz   )
);

// vme controller 8 bit interface at $ffff8e00 - $ffff8e0f
// (requierd to enable Mega STE cpu speed/cache control)
wire vme_sel = !steroids && mste && iodevice && ({mbus_a[15:4], 4'd0} == 16'h8e00);

/* ------------------------------------------------------------------------------ */
/* ---------------------------------- Blitter ----------------------------------- */
/* ------------------------------------------------------------------------------ */
wire        blitter_irq_n;
wire        blitter_br_n;
wire        blitter_bgack_n;
wire        blitter_bg_n;
wire        blitter_sel;
wire [15:0] blitter_data_out;

wire        blitter_as_n;
wire        blitter_ds_n;
wire        blitter_rw_n;
wire        blitter_fc0 = 1'b1, blitter_fc1 = 1'b0, blitter_fc2 = 1'b1;
wire        blitter_dtack_n;
wire [23:1] blitter_addr;
wire        blitter_has_bus;

blt_clks Clks;

assign Clks.clk = clk_32;
assign Clks.aRESETn = !peripheral_reset;
assign Clks.sReset = init | peripheral_reset;
assign Clks.pwrUp = init;

assign Clks.enPhi1 = use_16mhz ?  clk16_en : mhz8_en1;
assign Clks.enPhi2 = use_16mhz ? ~clk16_en : mhz8_en2;
assign Clks.anyPhi = Clks.enPhi2 | Clks.enPhi1;

assign { Clks.extReset, Clks.phi1, Clks.phi2} = 3'b000;

wire mblit_selected;
wire mblit_oBGACKn;

stBlitter stBlitter(
	.Clks     ( Clks ),
	.ASn      ( as_n | ~blitter_en ),
	.RWn      ( cpu_rw ),
	.LDSn     ( lds_n ),
	.UDSn     ( uds_n ),
	.FC0      ( fc0 ),
	.FC1      ( fc1 ),
	.FC2      ( fc2 ),
	.BERRn    ( berr_n ),
	.iDTACKn  ( dtack_n ),
	.ctrlOe   ( blitter_has_bus ),
	.dataOe   ( blitter_sel ),
	.oASn     ( blitter_as_n ),
	.oDSn     ( blitter_ds_n ),
	.oRWn     ( blitter_rw_n ),
	.oDTACKn  ( blitter_dtack_n ),
	.selected ( mblit_selected ),
	.iBRn     ( mcu_br_n ),
	.BGIn     ( blitter_bg_n ),
	.iBGACKn  ( mcu_bgack_n ),
	.oBRn     ( blitter_br_n ),
	.oBGACKn  ( mblit_oBGACKn ),
	.INTn     ( blitter_irq_n ),
	.BGOn     ( mcu_bg_n ),
	.dmaInput ( mbus_dout ),
	.iABUS    ( mbus_a ),
	.oABUS    ( blitter_addr ),
	.iDBUS    ( cpu_dout ),
	.oDBUS    ( blitter_data_out )
);

assign blitter_bgack_n = mblit_oBGACKn & mcu_bgack_n;		// This really happens inside Blitter
assign { blitter_fc2, blitter_fc1, blitter_fc0} = 3'b101;

/* ------------------------------------------------------------------------------ */
/* ---------------------------- STe controller ports ---------------------------- */
/* ------------------------------------------------------------------------------ */

wire [15:0] ste_joy_in;
wire  [3:0] ste_buttons;
reg   [7:0] ste_joy_out;

wire  [7:0] ste_joy_out_pins = joywe_n ? 8'hff : ste_joy_out;

always @(posedge clk_32) begin
	if (joywl) ste_joy_out <= mbus_dout[7:0];
end

ste_joypad ste_joypad0 (
	.joy      ( joy_port_ste ? joy1 : 16'd0 ),
	.din      ( ste_joy_out_pins[3:0] ),
	.dout     ( { ste_joy_in[11:8], ste_joy_in[3:0] } ),
	.buttons  ( ste_buttons[1:0] )
);

ste_joypad ste_joypad1 (
	.joy      ( joy_port_ste ? joy0 : 16'd0 ),
	.din      ( ste_joy_out_pins[7:4] ),
	.dout     ( { ste_joy_in[15:12], ste_joy_in[7:4] } ),
	.buttons  ( ste_buttons[3:2] )
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------- Real-time clock ------------------------------ */
/* ------------------------------------------------------------------------------ */

reg [3:0] rtc_data_out;
reg       rtc_bank;
reg [3:0] rtc_bank1[16];

// RP5C15
always @(*) begin
	casez ({rtc_bank, mbus_a[4:1]})
		5'b0_0000: rtc_data_out = rtc[3:0]; // sec
		5'b0_0001: rtc_data_out = rtc[7:4];
		5'b0_0010: rtc_data_out = rtc[11:8]; // min
		5'b0_0011: rtc_data_out = rtc[15:12];
		5'b0_0100: rtc_data_out = rtc[19:16]; // hour
		5'b0_0101: rtc_data_out = rtc[23:20];
		5'b0_0110: rtc_data_out = rtc[48:45]; // day of week
		5'b0_0111: rtc_data_out = rtc[27:24]; // day
		5'b0_1000: rtc_data_out = rtc[31:28];
		5'b0_1001: rtc_data_out = rtc[35:32]; // month
		5'b0_1010: rtc_data_out = rtc[39:36];
		5'b0_1011: rtc_data_out = rtc[43:40]; // year
		5'b0_1100: rtc_data_out = rtc[47:44] + 4'd2; // GEMDOS fix: it assumes year 0 as 1980
		5'b?_1101: rtc_data_out = {1'b1, 2'b00, rtc_bank };
		5'b?_1110: rtc_data_out = 4'h0;
		5'b?_1111: rtc_data_out = 4'hc;
		5'b1_????: rtc_data_out = rtc_bank1[mbus_a[4:1]];
		default: rtc_data_out = 4'hf;
	endcase
	// make the RTC invisible if no valid RTC data arrives from the IO Controller
	if (rtc == 0) rtc_data_out = 4'hf;
end

always @(posedge clk_32) begin
	if (peripheral_reset) begin
		rtc_bank <= 0;
	end else begin
		if (!(rtccs_n | rtcwr_n)) begin
			if (mbus_a[4:1] == 4'hd)
				rtc_bank <= cpu_dout[0];
			else
				rtc_bank1[mbus_a[4:1]] <= cpu_dout[3:0];
		end
	end
end

/* ------------------------------------------------------------------------------ */
/* ------------------------------------- DMA ------------------------------------ */
/* ------------------------------------------------------------------------------ */

wire dma_write, dma_read;
wire [15:0] dma_data_out;

wire acsi_irq;

dma dma (
	// system interface
	.clk          ( clk_32        ),
	.clk_en       ( mhz8_en1      ),
	.reset        ( reset         ),

	// cpu interface
	.cpu_din      ( mbus_dout     ),
	.cpu_sel      ( ~fcs_n        ),
	.cpu_a1       ( mbus_a[1]     ),
	.cpu_rw       ( rw            ),
	.cpu_dout     ( dma_data_out  ),

	// IO controller interface for ACSI
	.dio_data_in_strobe  ( data_in_strobe_acsi ),
	.dio_data_in_reg     ( data_in_reg         ),
	.dio_data_out_strobe ( data_out_strobe     ),
	.dio_data_out_reg    ( data_out_reg        ),
	.dio_dma_ack         ( dma_ack             ),
	.dio_dma_status      ( dma_status          ),
	.dio_dma_nak         ( dma_nak             ),
	.dio_status_in       ( dma_status_in       ),
	.dio_status_index    ( dma_status_index    ),

	// additional signals for ACSI interface
	.acsi_irq     ( acsi_irq           ),
	.acsi_enable  ( acsi_enable ),

	// FDC interface
	.fdc_drq      ( fdc_drq  ),
	.fdc_addr     ( fdc_addr ),
	.fdc_sel      ( fdc_sel  ),
	.fdc_rw       ( fdc_rw   ),
	.fdc_din      ( fdc_din  ),
	.fdc_dout     ( fdc_dout ),

	// ram interface
	.rdy_i        ( rdy_i        ),
	.rdy_o        ( rdy_o        ),
	.ram_din      ( shifter_dout )
);

assign     LED = (floppy_sel == 2'b11);
wire       fdc_irq;
wire       fdc_drq;
wire [1:0] fdc_addr;
wire       fdc_sel;
wire       fdc_rw;
wire [7:0] fdc_din;
wire [7:0] fdc_dout;

// Some broken software selects both drives at the same time. On real hardware this
// only works if no second drive is present. In our setup the second drive is present
// but we can simply map all such broken accesses to drive A only
wire [1:0] floppy_sel_exclusive = (floppy_sel == 2'b00)?2'b10:floppy_sel;

fdc1772 #(.IMG_TYPE(1)) fdc1772 (
	.clkcpu         ( clk_32           ), // system cpu clock.
	.clk8m_en       ( mhz8_en1         ),

	// external set signals
	.floppy_drive   ( {2'b11, floppy_sel_exclusive} ),
	.floppy_side    ( floppy_side      ),
	.floppy_reset   ( ~peripheral_reset),

	// interrupts
	.irq            ( fdc_irq          ),
	.drq            ( fdc_drq          ),

	.cpu_addr       ( fdc_addr         ),
	.cpu_sel        ( fdc_sel          ),
	.cpu_rw         ( fdc_rw           ),
	.cpu_din        ( fdc_din          ),
	.cpu_dout       ( fdc_dout         ),

	// place any signals that need to be passed up to the top after here.
	.img_mounted    ( img_mounted      ), // signaling that new image has been mounted
	.img_wp         ( img_wp           ), // write protect
	.img_size       ( img_size         ), // size of image in bytes
	.sd_lba         ( sd_lba           ),
	.sd_rd          ( sd_rd            ),
	.sd_wr          ( sd_wr            ),
	.sd_ack         ( sd_ack           ),
	.sd_buff_addr   ( sd_buff_addr     ),
	.sd_dout        ( sd_dout          ),
	.sd_din         ( sd_din           ),
	.sd_dout_strobe ( sd_dout_strobe   )
);

/* ------------------------------------------------------------------------------ */
/* --------------------- Ethernet on Cartridge Port (ETHERNEC) ------------------ */
/* ------------------------------------------------------------------------------ */

wire  [1:0] eth_rd = ~rom4_n & ethernec_present;
wire  [1:0] eth_wr = ~rom3_n & ethernec_present;
wire  [7:0] eth_data_out;

ethernec ethernec (
	.clk        (clk_32      ),
	.rd         (eth_rd      ),
	.wr         (eth_wr      ),
	.addr       (mbus_a[13:9]),
	.din        (mbus_a[8:1] ),
	.dout       (eth_data_out),

	.status     (eth_status),

	.mac_begin  (eth_mac_begin),
	.mac_strobe (eth_mac_strobe),
	.mac_byte   (eth_mac_byte),

	.tx_begin   (eth_tx_read_begin),
	.tx_strobe  (eth_tx_read_strobe),
	.tx_byte    (eth_tx_read_byte),

	.rx_begin   (eth_rx_write_begin),
	.rx_strobe  (eth_rx_write_strobe),
	.rx_byte    (eth_rx_write_byte)
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------- Cubase dongle  ------------------------------- */
/* ------------------------------------------------------------------------------ */
wire        cubase3_d8;
wire  [7:0] cubase2_dout;
wire  [7:0] cubase_dout = cubase_sel ? cubase2_dout : {7'h7f, cubase3_d8};
reg         cubase_sel; // Cubase3/2 dongle
reg         cubase_lock;

always @(posedge clk_32) begin
	if (peripheral_reset) begin
		cubase_sel <= 0;
		cubase_lock <= 0;
	end
	else if (cubase_enable & !rom3_n & !cubase_lock) begin
		cubase_sel <= |mbus_a[7:1];
		cubase_lock <= 1;
	end
end

cubase2_dongle cubase2_dongle (
	.clk        ( clk_32           ),
	.reset      ( peripheral_reset ),
	.uds_n      ( uds_n            ),
	.A          ( mbus_a[8:1]      ),
	.D          ( cubase2_dout     )
);

cubase3_dongle cubase3_dongle (
	.clk        ( clk_32           ),
	.reset      ( peripheral_reset ),
	.rom3_n     ( rom3_n           ),
	.a8         ( mbus_a[8]        ),
	.d8         ( cubase3_d8       )
);

/* ------------------------------------------------------------------------------ */
/* --------------------------- SDRAM bus multiplexer ---------------------------- */
/* ------------------------------------------------------------------------------ */

wire cpu_precycle = (bus_cycle == 0);
wire cpu_cycle    = (bus_cycle == 1) || (bus_cycle == 2 && turbo_bus);
wire viking_cycle = (bus_cycle == 2 && !turbo_bus) || (bus_cycle == 3 && turbo_bus); // this is the shifter cycle, too
wire viking_precycle = (bus_cycle == 3 && !turbo_bus) || (bus_cycle == 1 && turbo_bus);

reg ras_n_d;
reg data_wr;
wire ram_req = ras_n_d & ~ras_n & ~ram_ref; // RAS_N going low and not refresh
wire ram_we = ~ram_we_n;

// TOS/cartridge upload via data_io
reg tos192k = 1'b0;

always @(posedge clk_32) begin
	reg data_in_strobe_romD;

	ras_n_d <= ras_n;
	data_wr <= 1'b0;
	if (cpu_precycle && mhz8_en1) begin
		data_in_strobe_romD <= data_in_strobe_rom;
		if (data_in_strobe_rom ^ data_in_strobe_romD) data_wr <= 1'b1;
	end
	if (data_download) begin
		if (data_addr[23:18] == 6'b111111) tos192k <= 1'b1;
		else if (data_addr[23:20] == 4'he) tos192k <= 1'b0;
	end
end

// ----------------- RAM address --------------
wire [23:1] sdram_address = (cpu_cycle & data_download)?data_addr:
                            (viking_cycle & viking_active & viking_read)?viking_vaddr:ram_a;

wire        ram_en = (MEM512K & ram_a[23:19] == 5'b00000) ||
                     (MEM1M   & ram_a[23:20] == 4'b0000)  ||
                     (MEM2M   & ram_a[23:21] == 3'b000)   ||
                     (MEM4M   & ram_a[23:22] == 2'b00)    ||
                     (MEM8M   & ram_a[23] == 1'b0)        ||
                     (MEM14M  & (~ram_a[23] | ~ram_a[22] | (ram_a[23] & ram_a[22] & ~ram_a[21]))) ||
                     (viking_enable & ~steroids & ram_a[23:18] == 6'b110000) ||
                     (viking_enable &  steroids & ram_a[23:19] == 5'b11101);

// ----------------- RAM read -----------------
wire sdram_req = (cpu_cycle & data_download)?data_wr:
                 (viking_cycle & viking_active & viking_read)?1'b1:
                 (ram_req & ram_en);

// ----------------- RAM write -----------------
wire sdram_we = (cpu_cycle & data_download)?1'b1:ram_we;

wire [15:0] ram_data_in = data_download?data_in_reg:ram_din;

// data strobe
wire sdram_uds = (cpu_cycle & data_download)?1'b1:ram_uds;
wire sdram_lds = (cpu_cycle & data_download)?1'b1:ram_lds;

wire [23:1] rom_a = (!rom2_n & ~tos192k) ? { 4'hE, 2'b00, mbus_a[17:1] } :
                    (!rom2_n &  tos192k) ? { 4'hF, 2'b11, mbus_a[17:1] } : mbus_a;

wire [15:0] ram_data_out;
wire [63:0] ram_data_out64;
wire [15:0] rom_data_out;


sdram sdram (
	// interface to the MT48LC16M16 chip
	.sd_data     ( SDRAM_DQ                 ),
	.sd_addr     ( SDRAM_A                  ),
	.sd_dqm      ( {SDRAM_DQMH, SDRAM_DQML} ),
	.sd_cs       ( SDRAM_nCS                ),
	.sd_ba       ( SDRAM_BA                 ),
	.sd_we       ( SDRAM_nWE                ),
	.sd_ras      ( SDRAM_nRAS               ),
	.sd_cas      ( SDRAM_nCAS               ),

	// system interface
	.clk_96      ( clk_96                   ),
	.clk_8_en    ( mhz8_en1                 ),
	.init        ( init                     ),

	// cpu/chipset interface
	.din         ( ram_data_in              ),
	.addr        ( { 1'b0, sdram_address }  ),
	.ds          ( { sdram_uds, sdram_lds } ),
	.req         ( sdram_req                ),
	.we          ( sdram_we                 ),
	.dout        ( ram_data_out             ),
	.dout64      ( ram_data_out64           ),

	// ROM access port
	.rom_oe      ( ~rom_n                   ),
	.rom_addr    ( rom_a                    ),
	.rom_dout    ( rom_data_out             )
);

endmodule
