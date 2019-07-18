/********************************************/
/*                                          */
/********************************************/

module mist_top ( 
  // clock inputs	
  input wire [ 2-1:0] 	CLOCK_27, // 27 MHz
  // LED outputs
  output wire 		LED, // LED Yellow
  // UART
  output wire 		UART_TX, // UART Transmitter (MIDI out)
  input wire 		UART_RX, // UART Receiver (MIDI in)
  // VGA
  output wire 		VGA_HS, // VGA H_SYNC
  output wire 		VGA_VS, // VGA V_SYNC
  output wire [ 6-1:0] 	VGA_R, // VGA Red[5:0]
  output wire [ 6-1:0] 	VGA_G, // VGA Green[5:0]
  output wire [ 6-1:0] 	VGA_B, // VGA Blue[5:0]
  // SDRAM
  inout wire [ 16-1:0] 	SDRAM_DQ, // SDRAM Data bus 16 Bits
  output wire [ 13-1:0] SDRAM_A, // SDRAM Address bus 13 Bits
  output wire 		SDRAM_DQML, // SDRAM Low-byte Data Mask
  output wire 		SDRAM_DQMH, // SDRAM High-byte Data Mask
  output wire 		SDRAM_nWE, // SDRAM Write Enable
  output wire 		SDRAM_nCAS, // SDRAM Column Address Strobe
  output wire 		SDRAM_nRAS, // SDRAM Row Address Strobe
  output wire 		SDRAM_nCS, // SDRAM Chip Select
  output wire [ 2-1:0] 	SDRAM_BA, // SDRAM Bank Address
  output wire 		SDRAM_CLK, // SDRAM Clock
  output wire 		SDRAM_CKE, // SDRAM Clock Enable
  // MINIMIG specific
  output wire 		AUDIO_L, // sigma-delta DAC output left
  output wire 		AUDIO_R, // sigma-delta DAC output right
  // SPI
  inout wire 		SPI_DO,
  input wire 		SPI_DI,
  input wire 		SPI_SCK,
  input wire 		SPI_SS2,    // fpga
  input wire 		SPI_SS3,    // OSD
  input wire 		SPI_SS4,    // "sniff" mode
  input wire 		CONF_DATA0  // SPI_SS for user_io
);

// enable additional ste/megaste features
wire ste = 1;//system_ctrl[23] || system_ctrl[24];
wire mste = system_ctrl[24];
wire steroids = system_ctrl[23] && system_ctrl[24];  // a STE on steroids

// ethernec is enabled by the io controller whenever a USB 
// ethernet interface is detected
wire ethernec_present = system_ctrl[25];

// usb target port on io controller is used for redirection of
// 0=nothing 1=rs232 2=printer 3=midi
wire [1:0] usb_redirection = system_ctrl[27:26];

wire psg_stereo = system_ctrl[22];

// clock generation
wire pll_locked;
wire clk_32;
wire clk_96;

// use pll
clock clock (
  .inclk0       (CLOCK_27[0]      ), // input clock (27MHz)
  .c0           (clk_96           ), // output clock c0 (96MHz)
  .c1           (clk_32           ), // output clock c1 (32MHz)
  .locked       (pll_locked       )  // pll locked output
);
assign SDRAM_CLK = clk_96;
wire init = ~pll_locked;

// MFP clock
// required: 2.4576 MHz
wire clk_mfp;
pll_mfp1 pll_mfp1 (
  .inclk0       (CLOCK_27[0]      ), // input clock (27MHz)
  .c0           (clk_mfp          )  // output clock c0 (2.4576MHz)
);

wire reset = system_ctrl[0];


// MCU signals
reg         mcu_reset_n;
always @(posedge clk_32) begin
	reg resetD;
	mcu_reset_n <= 1;
	resetD <= reset;
	if (resetD & ~reset) mcu_reset_n <= 0;
end

wire        mhz4, mhz4_en;
wire        mcu_dtack_n;
wire        hsync_n, vsync_n;
wire        rom0_n, rom1_n, rom2_n, rom3_n, rom4_n, rom5_n, rom6_n, romp_n;
wire        ras0_n, ras1_n;
wire        mfpint_n, mfpcs_n, mfpiack_n;
wire        sndir, sndcs;
wire        n6850, fcs_n;
wire        rtccs_n, rtcrd_n, rtcwr_n;
wire        sint;
wire [15:0] mcu_dout;
wire        ras_n = ras0_n & ras1_n;

// compatibility for existing dma and blitter
wire  [1:0] bus_cycle;

// CPU signals
wire        mhz8, mhz8_en1, mhz8_en2;
wire        berr_n;
wire        dtack_n;
wire        ipl0_n, ipl1_n, ipl2_n;
wire        fc0, fc1, fc2;
wire        as_n, cpu_rw, uds_n, lds_n, vma_n, vpa_n, cpu_E;
wire [15:0] cpu_din, cpu_dout;
wire [23:1] cpu_a;

wire        rom_n = rom0_n & rom1_n & rom2_n & rom3_n & rom4_n & rom5_n & rom6_n & romp_n;
assign      cpu_din = 
              dma_sel ? dma_data_out :
				  blitter_sel ? blitter_data_out :
              !rdat_n  ? shifter_dout :
				  !(mfpcs_n & mfpiack_n)? { 8'hff, mfp_data_out } :
              !rom_n   ? rom_data_out :
              !n6850   ? { acia_data_out, 8'hFF } :
              sndcs    ? { snd_data_out, 8'hFF }:
              mcu_dout;

// Shifter signals
wire        cmpcs_n, latch, de, blank_n, rdat_n, wdat_n, dcyc_n, sreq, sload_n, mono;
wire [ 7:0] dma_snd_l, dma_snd_r;
wire [ 3:0] r, g, b;

// RAM signals
wire [23:1] ram_a;
wire        ram_uds, ram_lds, ram_we_n;
wire [15:0] ram_din;

gstmcu gstmcu (
	.clk32      ( clk_32 ),
	.resb       ( mcu_reset_n ),
	.porb       ( mcu_reset_n ),
	.BR_N       ( ~br ),
	.FC0        ( fc0 ),
	.FC1        ( fc1 ),
	.FC2        ( fc2 ),
	.AS_N       ( as_n ),
	.RW         ( cpu_rw ),
	.UDS_N      ( uds_n ),
	.LDS_N      ( lds_n ),
	.VMA_N      ( vma_n ),
	.MFPINT_N   ( mfpint_n ),
	.A          ( cpu_a ),  // from CPU
	.ADDR       ( ram_a ),  // to RAM
	.DIN        ( cpu_dout ),
	.DOUT       ( mcu_dout ),
	.MHZ8       ( mhz8 ),
   .MHZ8_EN1   ( mhz8_en1 ),
	.MHZ8_EN2   ( mhz8_en2 ),
	.MHZ4       ( mhz4 ),
	.MHZ4_EN    ( mhz4_en ),
	.BERR_N     ( berr_n ),
	.IPL0_N     ( ipl0_n ),
	.IPL1_N     ( ipl1_n ),
	.IPL2_N     ( ipl2_n ),
	.DTACK_N    ( mcu_dtack_n ),
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
	.bus_cycle  ( bus_cycle )
);

wire [15:0] shifter_dout;

gstshifter gstshifter (
	.clk32      ( clk_32 ),
	.ste        ( ste ),
	.resb       ( !reset ),

    // CPU/RAM interface
	.CS         ( ~cmpcs_n ),
	.A          ( cpu_a[6:1] ),
	.DIN        ( cpu_dout ),
	.DOUT       ( shifter_dout ),
	.LATCH      ( latch ),
	.RDAT_N     ( rdat_n ),   // latched MDIN -> DOUT
	.WDAT_N     ( wdat_n ),   // DIN  -> MDOUT
	.RW         ( cpu_rw ),
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

mist_video #(.OSD_COLOR(3'b010), .COLOR_DEPTH(4), .SD_HCNT_WIDTH(10)) mist_video(
	.clk_sys    ( clk_32 ),
	.SPI_SCK    ( SPI_SCK ),
	.SPI_SS3    ( SPI_SS3 ),
	.SPI_DI     ( SPI_DI ),
	.R          ( r ),
	.G          ( g ),
	.B          ( b ),
	.HSync      ( hsync_n ),
   .VSync      ( vsync_n ),
   .VGA_R      ( VGA_R ),
	.VGA_G      ( VGA_G ),
	.VGA_B      ( VGA_B ),
	.VGA_VS     ( VGA_VS ),
	.VGA_HS     ( VGA_HS ),
	.ce_divider ( 1'b1 ),
	.rotate     ( 2'b00 ),
	.scandoubler_disable( scandoubler_disable | mono ),
	.scanlines  ( system_ctrl[21:20] ),
	.ypbpr      ( ypbpr )
);

assign      dtack_n = (mcu_dtack_n & ~mfp_dtack & ~dma_sel & ~blitter_sel) | dma_br;

fx68k fx68k (
	.clk        ( clk_32 ),
	.extReset   ( reset ),
	.pwrUp      ( reset ),
	.enPhi1     ( mhz8_en1 ),
	.enPhi2     ( mhz8_en2 ),

	.eRWn       ( cpu_rw ),
	.ASn        ( as_n ),
	.LDSn       ( lds_n ),
	.UDSn       ( uds_n ),
	.E          ( cpu_E ),
	.VMAn       ( vma_n ),
	.FC0        ( fc0 ),
	.FC1        ( fc1 ),
	.FC2        ( fc2 ),
	.BGn        ( blitter_bg ),
	.oRESETn    (),
	.oHALTEDn   (),
	.DTACKn     ( dtack_n ),
	.VPAn       ( vpa_n ),
	.BERRn		( berr_n ),
	.BRn        ( ~blitter_br ),
	.BGACKn     ( ~blitter_bgack ),
	.IPL0n      ( ipl0_n ),
	.IPL1n      ( ipl1_n ),
	.IPL2n      ( ipl2_n ),
	.iEdb       ( cpu_din ),
	.oEdb       ( cpu_dout ),
	.eab        ( cpu_a )
);

wire acia_irq, dma_irq;

// the STE delays the xsirq by 1/250000 second before feeding it into timer_a
// 74ls164
wire      xsint = ~sint;
reg [7:0] xsint_delay;
always @(posedge clk_32 or negedge xsint) begin
	if(!xsint) xsint_delay <= 8'h00;            // async reset
	else if (clk_2_en) xsint_delay <= {xsint_delay[6:0], xsint};
end

wire xsint_delayed = xsint_delay[7];

// according to hatari video.h/video.c the timer_b irq comes 28 8MHz cycles after
// the last pixel has been displayed. There's no delaying on the STe schematics, but
// bottom border removing fails (set to 60Hz comes a bit early) without this delay.
// Maybe it's inside the MFP?
reg [27:0] st_de_delay;
wire st_de = st_de_delay[27];
always @(posedge clk_32)
   if (mhz8_en1) st_de_delay <= { st_de_delay[26:0], de };

// mfp io7 is mono_detect which in ste is xor'd with the dma sound irq
wire mfp_io7 = system_ctrl[8] ^ (ste?xsint:1'b0);

// input 0 is busy from printer which is pulled up when the printer cannot accept further data
// if no printer redirection is being used this is wired to the extra joystick ports provided
// by the "gauntlet2 adapter". If no extra joystick ports are present this will return 1
wire parallel_fifo_full;
wire mfp_io0 = (usb_redirection == 2'd2)?parallel_fifo_full:~joy0[4];

// inputs 1,2 and 6 are inputs from serial which have pullups before and inverter
wire  [7:0] mfp_gpio_in = {mfp_io7, 1'b0, !dma_irq, !acia_irq, !blitter_irq, 2'b00, mfp_io0};
wire  [1:0] mfp_timer_in = {st_de, ste?xsint_delayed:!parallel_fifo_full};
wire        mfp_dtack;
wire        mfp_int, mfp_iack = ~mfpiack_n;
assign      mfpint_n = ~mfp_int;
wire  [7:0] mfp_data_out;

mfp mfp (
	// cpu register interface
	.clk      ( clk_32        ),
	.clk_en   ( mhz4_en       ),
	.reset    ( reset         ),
	.din      ( cpu_dout[7:0] ),
	.sel      ( ~mfpcs_n      ),
	.addr     ( cpu_a[5:1]    ),
	.ds       ( lds_n         ),
	.rw       ( cpu_rw        ),
	.dout     ( mfp_data_out  ),
	.irq      ( mfp_int       ),
	.iack     ( mfp_iack      ),
	.dtack    ( mfp_dtack     ),

	// serial/rs232 interface io-controller<->mfp
	.serial_data_out_available 	(serial_data_from_mfp_available),
	.serial_strobe_out 				(serial_strobe_from_mfp),
	.serial_data_out 	   			(serial_data_from_mfp),
	.serial_status_out 	   		(serial_status_from_mfp),
	.serial_strobe_in 				(serial_strobe_to_mfp),
	.serial_data_in 	   			(serial_data_to_mfp),
	.serial_status_in 	   		(serial_status_to_mfp),

	// input signals
	.clk_ext  ( clk_mfp       ),  // 2.457MHz clock
	.t_i      ( mfp_timer_in  ),  // timer a/b inputs
	.i        ( mfp_gpio_in   )   // gpio-in
);

wire [7:0] acia_data_out;

acia acia (
	// cpu interface
	.clk      ( clk_32        ),
	.E        ( cpu_E         ),
	.reset    ( reset         ),
	.din      ( cpu_dout[15:8]),
	.sel      ( ~n6850        ),
	.addr     ( cpu_a[2:1]    ),
	.rw       ( cpu_rw        ),
	.dout     ( acia_data_out ),
	.irq      ( acia_irq      ),

	// physical MIDI interface
	.midi_out ( UART_TX       ),
	.midi_in  ( UART_RX       ),
	 
	// ikbd interface
	.ikbd_data_out_available 	(ikbd_data_from_acia_available),
	.ikbd_strobe_out 				(ikbd_strobe_from_acia),
	.ikbd_data_out 	   		(ikbd_data_from_acia),
	.ikbd_strobe_in 				(ikbd_strobe_to_acia),
	.ikbd_data_in 	   			(ikbd_data_to_acia),

	// redirected midi interface
	.midi_data_out_available 	(midi_data_from_acia_available),
	.midi_strobe_out 				(midi_strobe_from_acia),
	.midi_data_out 	   		(midi_data_from_acia)
);

wire [7:0] snd_data_out;
wire [7:0] ym_a_out, ym_b_out, ym_c_out;

wire [9:0] ym_audio_out_l = psg_stereo ? ym_a_out + ym_b_out : ym_a_out + ym_b_out + ym_c_out;
wire [9:0] ym_audio_out_r = psg_stereo ? ym_c_out + ym_b_out : ym_a_out + ym_b_out + ym_c_out;

reg clk_2_en;
always @(posedge clk_32) begin
	reg [3:0] cnt;
	clk_2_en <= (cnt == 0);
	cnt <= cnt + 1'd1;
end

// extra joysticks are wired to the printer port
// using the "gauntlet2 interface", fire of
// joystick 0 is connected to the mfp I0 (busy)
wire [7:0] port_b_in = { ~joy0[0], ~joy0[1], ~joy0[2], ~joy0[3],~joy1[0], ~joy1[1], ~joy1[2], ~joy1[3]};
wire [7:0] port_a_in = { 2'b11, ~joy1[4], 5'b11111 };
wire [7:0] port_a_out;
wire [7:0] port_b_out;
wire       floppy_side = port_a_out[0];
wire [1:0] floppy_sel = port_a_out[2:1];

ym2149 ym2149 (
	.CLK         ( clk_32        ),
	.CE          ( clk_2_en      ),
	.RESET       ( reset         ),
	.DI          ( cpu_dout[15:8]),
	.DO          ( snd_data_out  ),
	.CHANNEL_A   ( ym_a_out      ),
	.CHANNEL_B   ( ym_b_out      ),
	.CHANNEL_C   ( ym_c_out      ),
	.BDIR        ( sndir         ),
	.BC          ( sndcs         ),
	.MODE        ( 0             ),
	.SEL         ( 0             ),
	.IOA_in      ( port_a_in     ),
	.IOA_out     ( port_a_out    ),
	.IOB_in      ( port_b_in     ),
	.IOB_out     ( port_b_out    )
);

// ------ fifo to store printer data coming from psg ---------
io_fifo #(.DEPTH(4)) parallel_out_fifo (
	.reset          ( reset ),

	.in_clk         ( clk_32 ),
	.in             ( port_b_out ),
	.in_strobe      ( port_a_out[5] ),
	.in_enable      ( 1'b0 ),

	.out_clk        ( clk_32 ),
	.out            ( parallel_data_out ),
	.out_strobe     ( parallel_strobe_out ),
	.out_enable     ( 1'b0 ),

	.full           ( parallel_fifo_full ),
	.data_available ( parallel_data_out_available )
);
// audio output processing

// YM and STE audio channels are expanded to 14 bits and added resulting in 15 bits
// for the sigmadelta dac take from the minimig

// This should later be handled by the lmc1992

wire [9:0] ym_audio_out_l_signed = ym_audio_out_l - 10'h200;
wire [9:0] ym_audio_out_r_signed = ym_audio_out_r - 10'h200;
wire [7:0] ste_audio_out_l_signed = dma_snd_l - 8'h80;
wire [7:0] ste_audio_out_r_signed = dma_snd_r - 8'h80;

wire [14:0] audio_mix_l =
        { ym_audio_out_l_signed[9], ym_audio_out_l_signed, ym_audio_out_l_signed[9:6]} +
        { ste_audio_out_l_signed[7], ste_audio_out_l_signed, ste_audio_out_l_signed[7:2] };
wire [14:0] audio_mix_r =
        { ym_audio_out_r_signed[9], ym_audio_out_r_signed, ym_audio_out_r_signed[9:6]} +
        { ste_audio_out_r_signed[7], ste_audio_out_r_signed, ste_audio_out_r_signed[7:2] };

sigma_delta_dac sigma_delta_dac (
	.clk      ( clk_32      ),      // bus clock
	.ldatasum ( audio_mix_l ),      // left channel data
	.rdatasum ( audio_mix_r ),      // right channel data
	.left     ( AUDIO_L     ),      // left bitstream output
	.right    ( AUDIO_R     )       // right bitsteam output
);

wire [23:1] blitter_master_addr;
wire blitter_master_write;
wire blitter_master_read;
wire blitter_irq;
wire blitter_br;
wire blitter_bgack;
wire blitter_bg;
wire [15:0] blitter_master_data_out;
// blitter 16 bit interface at $ff8a00 - $ff8a3f, STE always has a blitter
wire blitter_sel = (system_ctrl[19] || ste) && cpu_a[23:16] == 8'hff && ~as_n && ~(uds_n && lds_n) && ({cpu_a[15:6], 6'd0} == 16'h8a00);
wire [15:0] blitter_data_out;

blitter blitter (
	// cpu interface
	.clk         ( clk_32           ),
	.clk_en      ( mhz8_en2         ),
	.reset       ( reset            ),
	.din         ( cpu_dout         ),
	.sel         ( blitter_sel      ),
	.addr        ( cpu_a[5:1]       ),
	.uds         ( uds_n            ),
	.lds         ( lds_n            ),
	.rw          ( cpu_rw           ),
	.dout        ( blitter_data_out ),

	.bus_cycle   ( bus_cycle               ),
	.bm_addr     ( blitter_master_addr     ),
	.bm_write    ( blitter_master_write    ),
	.bm_data_out ( blitter_master_data_out ),
	.bm_read     ( blitter_master_read     ),
	.bm_data_in  ( ram_data_out            ),

	.br_in       ( dma_br        ),
	.br_out      ( blitter_br    ),
	.bg          ( ~blitter_bg   ),
	.irq         ( blitter_irq   ),
	.bgack       ( blitter_bgack ),

	.turbo       ( 0             )
);

wire        dio_addr_strobe;
wire [31:0] dio_addr_reg;
wire        dio_data_in_strobe_uio;
wire        dio_data_in_strobe_mist;
wire [15:0] dio_data_in_reg;
wire        dio_data_out_strobe;
wire [15:0] dio_data_out_reg;
wire        dio_dma_ack;
wire  [7:0] dio_dma_status;
wire        dio_dma_nak;
wire  [7:0] dio_status_in;
wire  [4:0] dio_status_index;
wire [23:1] dio_data_addr;
wire        dio_download;

data_io data_io (
	.sck             ( SPI_SCK             ),
	.ss			     ( SPI_SS2             ),
	.sdi   		     ( SPI_DI              ),
	.sdo             ( dio_sdo             ),
	.clk             ( clk_32              ),
	.ctrl_out        ( system_ctrl         ),
	.video_adj	     ( video_adj           ),
	.addr_strobe     ( dio_addr_strobe     ),
	.addr_reg        ( dio_addr_reg        ),
	.data_in_strobe_uio ( dio_data_in_strobe_uio ),
	.data_in_strobe_mist( dio_data_in_strobe_mist),
	.data_in_reg     ( dio_data_in_reg     ),
	.data_addr       ( dio_data_addr       ),
	.data_download   ( dio_download        ),
	.data_out_strobe ( dio_data_out_strobe ),
	.data_out_reg    ( dio_data_out_reg    ),
	.dma_ack         ( dio_dma_ack         ),
	.dma_status      ( dio_dma_status      ),
	.dma_nak         ( dio_dma_nak         ),
	.status_in       ( dio_status_in       ),
	.status_index    ( dio_status_index    )
);

// floppy_sel is active low
wire wr_prot = (floppy_sel == 2'b01)?system_ctrl[7]:system_ctrl[6];

wire [22:0] dma_addr;
wire [15:0] dma_dout;
wire dma_write, dma_read;
wire dma_br;
wire dma_sel  = cpu_a[23:16] == 8'hff && ~as_n && ~(uds_n && lds_n) && (
         ~fcs_n ||                                             // ff8604-ff8607 word only
         ({cpu_a[15:2], 2'd0} == 16'h8608) ||                  // ff8608-ff860b
         ({cpu_a[15:1], 1'd0} == 16'h860c) ||                  // ff860c-ff860d
        (({cpu_a[15:1], 1'd0} == 16'h860e) && ste));           // ff860e-ff860f (hdmode, STE)
wire [15:0] dma_data_out;

dma dma (
	// system interface
	.clk        ( clk_32        ),
	.clk_en     ( mhz8_en1      ),
	.reset    	( reset       	 ),
	.bus_cycle  ( bus_cycle     ),
	.irq      	( dma_irq     	 ),

	// IO controller interface
	.dio_addr_strobe     ( dio_addr_strobe     ),
	.dio_addr_reg        ( dio_addr_reg        ),
	.dio_data_in_strobe  ( dio_data_in_strobe_mist ),
	.dio_data_in_reg     ( dio_data_in_reg     ),
	.dio_data_out_strobe ( dio_data_out_strobe ),
	.dio_data_out_reg    ( dio_data_out_reg    ),
	.dio_dma_ack         ( dio_dma_ack         ),
	.dio_dma_status      ( dio_dma_status      ),
	.dio_dma_nak         ( dio_dma_nak         ),
	.dio_status_in       ( dio_status_in       ),
	.dio_status_index    ( dio_status_index    ),

	// cpu interface
	.cpu_din      ( cpu_dout      ),
	.cpu_sel      ( dma_sel       ),
	.cpu_addr     ( cpu_a[3:1]    ),
	.cpu_uds      ( uds_n         ),
	.cpu_lds      ( lds_n         ),
	.cpu_rw       ( cpu_rw        ),
	.cpu_dout     ( dma_data_out  ),

	// additional signals for floppy/acsi interface
	.fdc_wr_prot  ( wr_prot       ),
	.drv_sel      ( floppy_sel    ),
	.drv_side     ( floppy_side   ),
	.acsi_enable  ( system_ctrl[17:10] ),

	// ram interface
	.ram_br        ( dma_br       ),
	.ram_read 	 	( dma_read	   ),
	.ram_write 	 	( dma_write	   ),
	.ram_addr		( dma_addr	   ),
	.ram_dout 		( dma_dout 	   ),
	.ram_din  		( ram_data_out	)
);

assign LED = (floppy_sel == 2'b11);


/* ------------------------------------------------------------------------------ */
/* --------------------------- SDRAM bus multiplexer ---------------------------- */
/* ------------------------------------------------------------------------------ */

wire dma_has_bus = dma_br;
wire blitter_has_bus = blitter_bgack;

// no tristate busses exist inside the FPGA. so bus request doesn't do
// much more than halting the cpu by suppressing 
wire br = dma_br || blitter_bgack; // dma/blitter are only other bus masters

wire cpu_cycle   = (bus_cycle == 1);

reg ras_n_d;
reg data_wr;
wire ram_oe = ras_n_d & ~ras_n & ram_we_n & |ram_a;
wire ram_we = ras_n_d & ~ras_n & ~ram_we_n;

always @(posedge clk_32) begin
	reg dio_data_in_strobe_uioD;

	ras_n_d <= ras_n;
	data_wr <= 1'b0;
	if (bus_cycle == 0 && mhz8_en1) begin
		dio_data_in_strobe_uioD <= dio_data_in_strobe_uio;
		if (dio_data_in_strobe_uio ^ dio_data_in_strobe_uioD) data_wr <= 1'b1;
	end
end

// ----------------- RAM address --------------
wire [23:1] dma_address = dma_has_bus?dma_addr:blitter_master_addr;
wire [23:1] sdram_address = (cpu_cycle & dio_download)?dio_data_addr:(cpu_cycle & br)?dma_address:ram_a;

// ----------------- RAM read -----------------
wire dma_oe = dma_has_bus?dma_read:blitter_master_read;
wire sdram_oe = (cpu_cycle & dio_download)?1'b0:(cpu_cycle & br)?dma_oe:ram_oe;

// ----------------- RAM write -----------------
wire dma_we = dma_has_bus?dma_write:blitter_master_write;
wire sdram_we = (cpu_cycle & dio_download)?data_wr:(cpu_cycle & br)?dma_we:ram_we;

wire [15:0] ram_data_in = dio_download?dio_data_in_reg:dma_has_bus?dma_dout:(blitter_has_bus?blitter_master_data_out:ram_din);

// data strobe
wire sdram_uds = (cpu_cycle & (br | dio_download))?1'b1:ram_uds;
wire sdram_lds = (cpu_cycle & (br | dio_download))?1'b1:ram_lds;

wire [23:1] rom_a = !rom2_n ? { 4'hE, 2'b00, cpu_a[17:1] } : cpu_a;
wire [15:0] ram_data_out;
wire [63:0] ram_data_out_64;
wire [15:0] rom_data_out;

assign SDRAM_CKE = 1'b1;

sdram sdram (
	// interface to the MT48LC16M16 chip
	.sd_data     	( SDRAM_DQ                 ),
	.sd_addr     	( SDRAM_A                  ),
	.sd_dqm      	( {SDRAM_DQMH, SDRAM_DQML} ),
	.sd_cs       	( SDRAM_nCS                ),
	.sd_ba       	( SDRAM_BA                 ),
	.sd_we       	( SDRAM_nWE                ),
	.sd_ras      	( SDRAM_nRAS               ),
	.sd_cas      	( SDRAM_nCAS               ),

	// system interface
	.clk_96        ( clk_96                   ),
	.clk_8_en      ( mhz8_en1                 ),
	.init         	( init                     ),

	// cpu/chipset interface
	.din           ( ram_data_in              ),
	.addr          ( { 1'b0, sdram_address }  ),
	.ds            ( { sdram_uds, sdram_lds } ),
	.we            ( sdram_we                 ),
	.oe            ( sdram_oe                 ),
	.dout          ( ram_data_out             ),
	.dout64        ( ram_data_out_64          ),

	.rom_oe        ( ~rom_n                   ),
	.rom_addr      ( rom_a                    ),
	.rom_dout      ( rom_data_out             )
);

// multiplex spi_do, drive it from user_io if that's selected, drive
// it from data_io if it's selected and leave it open else (also
// to be able to monitor sd card data directly)
wire user_io_sdo, dio_sdo;

assign SPI_DO = (CONF_DATA0 == 1'b0)?user_io_sdo:
	((SPI_SS2 == 1'b0)?dio_sdo:1'bZ);

wire [31:0] system_ctrl;

// connection to transfer ikbd data from io controller to acia
wire [7:0] ikbd_data_to_acia;
wire ikbd_strobe_to_acia;

// connection to transfer ikbd data from acia to io controller
wire [7:0] ikbd_data_from_acia;
wire ikbd_strobe_from_acia;
wire ikbd_data_from_acia_available;

// connection to transfer midi data from acia to io controller
wire [7:0] midi_data_from_acia;
wire midi_strobe_from_acia;
wire midi_data_from_acia_available;

// connection to transfer serial/rs232 data from mfp to io controller
wire [7:0] serial_data_from_mfp;
wire serial_strobe_from_mfp;
wire serial_data_from_mfp_available;
wire [7:0] serial_data_to_mfp;
wire serial_strobe_to_mfp;
wire [7:0] serial_status_to_mfp;
wire [63:0] serial_status_from_mfp;

// connection to transfer parallel data from psg to io controller
wire [7:0] parallel_data_out;
wire parallel_strobe_out;
wire parallel_data_out_available;

// extra joystick interface
wire [5:0] joy0, joy1;

// connection between io controller and ethernet controller
//   mac address transfer io controller -> ethernec
wire [31:0] eth_status;
wire [7:0] eth_mac_byte;
wire eth_mac_strobe, eth_mac_begin;
wire [7:0] eth_tx_read_byte;
wire eth_tx_read_strobe, eth_tx_read_begin;
wire [7:0] eth_rx_write_byte;
wire eth_rx_write_strobe, eth_rx_write_begin;

wire [2:0] switches;
wire scandoubler_disable;
wire ypbpr;

//// user io has an extra spi channel outside minimig core ////
user_io user_io(
		// the spi interface
		.SPI_CLK                	(SPI_SCK),
		.SPI_SS_IO						(CONF_DATA0),
		.SPI_MISO						(user_io_sdo),
		.SPI_MOSI						(SPI_DI),
		
		// ikbd interface
      .ikbd_strobe_out				(ikbd_strobe_from_acia),
      .ikbd_data_out					(ikbd_data_from_acia),
      .ikbd_data_out_available	(ikbd_data_from_acia_available),
      .ikbd_strobe_in				(ikbd_strobe_to_acia),
      .ikbd_data_in					(ikbd_data_to_acia),

		// extra joysticks
		.joy0                  		(joy0),
		.joy1                  		(joy1),
		
		// serial/rs232 interface
      .serial_strobe_out			(serial_strobe_from_mfp),
      .serial_data_out				(serial_data_from_mfp),
      .serial_data_out_available	(serial_data_from_mfp_available),
      .serial_status_out			(serial_status_from_mfp),
      .serial_strobe_in				(serial_strobe_to_mfp),
      .serial_data_in				(serial_data_to_mfp),
      .serial_status_in				(serial_status_to_mfp),
		
		// parallel interface
      .parallel_strobe_out			(parallel_strobe_out),
      .parallel_data_out			(parallel_data_out),
      .parallel_data_out_available(parallel_data_out_available),

		// midi interface
      .midi_strobe_out				(midi_strobe_from_acia),
      .midi_data_out					(midi_data_from_acia),
      .midi_data_out_available	(midi_data_from_acia_available),

		// ethernet
		.eth_status                (eth_status),
		.eth_mac_begin					(eth_mac_begin),
		.eth_mac_strobe				(eth_mac_strobe),
		.eth_mac_byte					(eth_mac_byte),
		.eth_tx_read_begin			(eth_tx_read_begin),
		.eth_tx_read_strobe			(eth_tx_read_strobe),
		.eth_tx_read_byte				(eth_tx_read_byte),
		.eth_rx_write_begin			(eth_rx_write_begin),
		.eth_rx_write_strobe			(eth_rx_write_strobe),
		.eth_rx_write_byte			(eth_rx_write_byte),

		// io controller requests to disable vga scandoubler
		.scandoubler_disable       (scandoubler_disable),
		.ypbpr                     (ypbpr),
		.SWITCHES                  (switches ),
		.CORE_TYPE                 (8'ha7)    // mist2 core id
);

			
endmodule

