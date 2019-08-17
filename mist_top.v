/********************************************/
/*       Atari ST/STe/Mega STe core         */
/********************************************/

module mist_top ( 
	// clock inputs
	input wire   [ 2-1:0] CLOCK_27,   // 27 MHz
	// LED outputs
	output wire           LED,        // LED Yellow
	// UART
	output wire           UART_TX,    // UART Transmitter (MIDI out)
	input wire            UART_RX,    // UART Receiver (MIDI in)
	// VGA
	output wire           VGA_HS,     // VGA H_SYNC
	output wire           VGA_VS,     // VGA V_SYNC
	output wire  [ 6-1:0] VGA_R,      // VGA Red[5:0]
	output wire  [ 6-1:0] VGA_G,      // VGA Green[5:0]
	output wire  [ 6-1:0] VGA_B,      // VGA Blue[5:0]
	// SDRAM
	inout wire  [ 16-1:0] SDRAM_DQ,   // SDRAM Data bus 16 Bits
	output wire [ 13-1:0] SDRAM_A,    // SDRAM Address bus 13 Bits
	output wire           SDRAM_DQML, // SDRAM Low-byte Data Mask
	output wire           SDRAM_DQMH, // SDRAM High-byte Data Mask
	output wire           SDRAM_nWE,  // SDRAM Write Enable
	output wire           SDRAM_nCAS, // SDRAM Column Address Strobe
	output wire           SDRAM_nRAS, // SDRAM Row Address Strobe
	output wire           SDRAM_nCS,  // SDRAM Chip Select
	output wire  [ 2-1:0] SDRAM_BA,   // SDRAM Bank Address
	output wire           SDRAM_CLK,  // SDRAM Clock
	output wire           SDRAM_CKE,  // SDRAM Clock Enable
	// AUDIO
	output wire           AUDIO_L,    // sigma-delta DAC output left
	output wire           AUDIO_R,    // sigma-delta DAC output right
	// SPI
	inout wire            SPI_DO,
	input wire            SPI_DI,
	input wire            SPI_SCK,
	input wire            SPI_SS2,    // fpga
	input wire            SPI_SS3,    // OSD
	input wire            SPI_SS4,    // "sniff" mode
	input wire            CONF_DATA0  // SPI_SS for user_io
);

// enable additional ste/megaste features
wire ste = system_ctrl[23] || system_ctrl[24];
wire mste = system_ctrl[24];
wire steroids = system_ctrl[23] && system_ctrl[24];  // a STE on steroids

// ethernec is enabled by the io controller whenever a USB 
// ethernet interface is detected
wire ethernec_present = system_ctrl[25];

// usb target port on io controller is used for redirection of
// 0=nothing 1=rs232 2=printer 3=midi
wire [1:0] usb_redirection = system_ctrl[27:26];

wire psg_stereo = system_ctrl[22];

// RAM size selects
wire MEM512K = (system_ctrl[3:1] == 3'd0);
wire MEM1M   = (system_ctrl[3:1] == 3'd1);
wire MEM2M   = (system_ctrl[3:1] == 3'd2);
wire MEM4M   = (system_ctrl[3:1] == 3'd3);
wire MEM8M   = (system_ctrl[3:1] == 3'd4);
wire MEM14M  = (system_ctrl[3:1] == 3'd5);

// clock generation
wire pll_locked;
wire clk_2;
wire clk_32;
wire clk_96;
wire clk_128;

// 32.084 MHz base clock
wire mainclock;
clock32 clock32 (
	.inclk0     (CLOCK_27[0]),
	.c0         (mainclock  )
);

clock clock (
  .inclk0       (mainclock  ), // input clock (32.084MHz)
  .c0           (clk_96     ), // output clock c0 (96MHz)
  .c1           (clk_32     ), // output clock c1 (32MHz)
  .c2           (clk_128    ), // output clock c2 (128MHz)
  .c3           (clk_2      ), // output clock c3 (2MHz)
  .locked       (pll_locked )  // pll locked output
);
wire init = ~pll_locked;

assign SDRAM_CLK = clk_96;

// MFP clock
// required: 2.4576 MHz
wire clk_mfp;
pll_mfp1 pll_mfp1 (
  .inclk0       (CLOCK_27[0]), // input clock (27MHz)
  .c0           (clk_mfp    )  // output clock c0 (2.4576MHz)
);


// registered reset signals
reg         reset;
reg         ikbd_reset;
reg         mcu_reset_n;

always @(posedge clk_32) begin
	reg resetD;

	reset <= system_ctrl[0];

	mcu_reset_n <= 1;
	resetD <= reset;
	if (~resetD & reset) mcu_reset_n <= 0;
end

always @(posedge clk_2) ikbd_reset <= system_ctrl[0];

// MCU signals

wire        mhz4, mhz4_en, clk16, clk16_en = ~clk16;
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

// dma
wire        rdy_o, rdy_i, mcu_bg_n = blitter_bg_n & ~blitter_bgack, mcu_br_n, mcu_bgack_n;

// compatibility for existing blitter
wire  [1:0] bus_cycle;

// for other peripherals
wire        iodevice = ~as_n & fc2 & (fc0 ^ fc1) & cpu_a[23:16] == 8'hff;

// CPU signals
wire        mhz8, mhz8_en1, mhz8_en2;
wire        berr_n;
wire        cpu_dtack_n;
wire        ipl0_n, ipl1_n, ipl2_n;
wire        fc0, fc1, fc2;
wire        as_n, cpu_rw, uds_n, lds_n, vma_n, vpa_n, cpu_E;
wire [15:0] cpu_din, cpu_dout;
wire [23:1] cpu_a;

wire        rom_n = rom0_n & rom1_n & rom2_n & rom3_n & rom4_n & rom5_n & rom6_n & romp_n;
assign      cpu_din = 
              ~fcs_n ? dma_data_out :
              blitter_sel ? blitter_data_out :
              !rdat_n  ? shifter_dout :
              !(mfpcs_n & mfpiack_n)? { 8'hff, mfp_data_out } :
              !rom_n   ? rom_data_out :
              n6850    ? { cpu_a[2] ? midi_acia_data_out : kbd_acia_data_out, 8'hFF } :
              sndcs    ? { snd_data_out, 8'hFF }:
              mste_ctrl_sel ? {8'hff, mste_ctrl_data_out }:
              mcu_dout;

wire [15:0] mbus_dout = ~rdy_i ? dma_data_out : cpu_dout; // dout from the current bus master

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
	.CLK_O      ( clk16 ),
	.MHZ8       ( mhz8 ),
	.MHZ8_EN1   ( mhz8_en1 ),
	.MHZ8_EN2   ( mhz8_en2 ),
	.MHZ4       ( mhz4 ),
	.MHZ4_EN    ( mhz4_en ),
	.RDY_N_I    ( rdy_o ),
	.RDY_N_O    ( rdy_i ),
	.BG_N       ( mcu_bg_n ),
	.BR_N_I     ( ~blitter_br ),
	.BR_N_O     ( mcu_br_n ),
	.BGACK_N_I  ( ~blitter_bgack ),
	.BGACK_N_O  ( mcu_bgack_n ),
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

	.st            ( ~ste ),
	.extra_ram     ( MEM8M | MEM14M ),
	.tos192k       ( tos192k ),
	.viking_at_c0  ( viking_enable && !steroids ),
	.viking_at_e8  ( viking_enable &&  steroids ),
	.bus_cycle     ( bus_cycle )
);

wire [15:0] shifter_dout;

gstshifter gstshifter (
	.clk32      ( clk_32 ),
	.ste        ( ste ),
	.resb       ( mcu_reset_n ),

    // CPU/RAM interface
	.CS         ( ~cmpcs_n ),
	.A          ( cpu_a[6:1] ),
	.DIN        ( mbus_dout ),
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

// --------------- the Viking compatible 1280x1024 graphics card -----------------

// viking/sm194 is enabled and max 8MB memory may be enabled. In steroids mode
// video memory is moved to $e80000 and all stram up to 14MB may be used
wire viking_mem_ok = MEM512K || MEM1M || MEM2M || MEM4M || MEM8M;
wire viking_enable = (system_ctrl[28] && viking_mem_ok) || steroids;

// check for cpu access to 0xcxxxxx with viking enabled to switch video
// output once the driver loads. 256 accesses to the viking memory range
// are considered a valid sign that the driver is working. Without driver
// others may also probe that area which is why we want to see 256 accesses
reg [7:0] viking_in_use;
reg       viking_active;

always @(posedge clk_32) begin
	if(reset) begin
		viking_in_use <= 8'h00;
		viking_active <= 1'b0;
	end else begin
		// cpu writes to $c0xxxx or $e80000
		if(mhz8_en1 && !as_n && viking_enable &&
		  (cpu_a[23:18] == (steroids?6'b111010:6'b110000)) && (viking_in_use != 8'hff))
			viking_in_use <= viking_in_use + 1'd1;

		viking_active <= (viking_in_use == 8'hff);
	end
end

wire viking_hs, viking_vs;
wire [3:0] viking_r, viking_g, viking_b;

wire [23:1] viking_vaddr;
wire viking_read;

viking viking (
	.pclk      ( clk_128         ), // 128MHz
	.himem     ( steroids        ),
	.clk_8_en  ( mhz8_en1        ), // 8 MHz bus clock
	.bus_cycle ( bus_cycle       ), // bus-cycle to sync video memory access with cpu

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

wire       video_clk = viking_active ? clk_128 : clk_32;
wire [3:0] stvid_r   = viking_active?viking_r:r;
wire [3:0] stvid_g   = viking_active?viking_g:g;
wire [3:0] stvid_b   = viking_active?viking_b:b;
wire       stvid_hs  = viking_active?viking_hs:hsync_n;
wire       stvid_vs  = viking_active?viking_vs:vsync_n;

// assume mono mode only if it's set during VSYNC
// demos like to switch it on/off during active display to get rid of borders
reg        monomode = 1'b0;
always @(posedge clk_32) begin
	if (!vsync_n) monomode <= mono;
end

mist_video #(.OSD_COLOR(3'b010), .COLOR_DEPTH(4), .SD_HCNT_WIDTH(10)) mist_video(
	.clk_sys    ( video_clk ),
	.SPI_SCK    ( SPI_SCK ),
	.SPI_SS3    ( SPI_SS3 ),
	.SPI_DI     ( SPI_DI ),
	.R          ( stvid_r ),
	.G          ( stvid_g ),
	.B          ( stvid_b ),
	.HSync      ( stvid_hs ),
	.VSync      ( stvid_vs ),
	.VGA_R      ( VGA_R ),
	.VGA_G      ( VGA_G ),
	.VGA_B      ( VGA_B ),
	.VGA_VS     ( VGA_VS ),
	.VGA_HS     ( VGA_HS ),
	.ce_divider ( 1'b1 ),
	.rotate     ( 2'b00 ),
	.scandoubler_disable( scandoubler_disable | monomode | viking_active ),
	.no_csync   ( monomode | viking_active ),
	.scanlines  ( system_ctrl[21:20] ),
	.ypbpr      ( ypbpr )
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------------ CPU ------------------------------------- */
/* ------------------------------------------------------------------------------ */

assign      cpu_dtack_n = mcu_dtack_n_adj & ~mfp_dtack & ~mste_ctrl_sel & ~vme_sel & ~blitter_sel;

reg         use_16mhz;
always @(posedge clk_32) if (mhz8_en2) use_16mhz <= (enable_16mhz | steroids);
wire        fx68_phi1 = use_16mhz ?  clk16_en : mhz8_en1;
wire        fx68_phi2 = use_16mhz ? ~clk16_en : mhz8_en2;

wire        mcu_dtack_n_adj = (use_16mhz & ~rom_n) ? (mcu_dtack_n | bus_cycle == 2'd2) : mcu_dtack_n;

fx68k fx68k (
	.clk        ( clk_32 ),
	.extReset   ( reset ),
	.pwrUp      ( reset ),
	.enPhi1     ( fx68_phi1 ),
	.enPhi2     ( fx68_phi2 ),

	.eRWn       ( cpu_rw ),
	.ASn        ( as_n ),
	.LDSn       ( lds_n ),
	.UDSn       ( uds_n ),
	.E          ( cpu_E ),
	.VMAn       ( vma_n ),
	.FC0        ( fc0 ),
	.FC1        ( fc1 ),
	.FC2        ( fc2 ),
	.BGn        ( blitter_bg_n ),
	.oRESETn    (),
	.oHALTEDn   (),
	.DTACKn     ( cpu_dtack_n ),
	.VPAn       ( vpa_n ),
	.BERRn		( berr_n ),
	.BRn        ( ~blitter_br & mcu_br_n ),
	.BGACKn     ( ~blitter_bgack & mcu_bgack_n ),
	.IPL0n      ( ipl0_n ),
	.IPL1n      ( ipl1_n ),
	.IPL2n      ( ipl2_n ),
	.iEdb       ( cpu_din ),
	.oEdb       ( cpu_dout ),
	.eab        ( cpu_a )
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
wire mfp_io7 = system_ctrl[8] ^ (ste?xsint:1'b0);

// input 0 is busy from printer which is pulled up when the printer cannot accept further data
// if no printer redirection is being used this is wired to the extra joystick ports provided
// by the "gauntlet2 adapter". If no extra joystick ports are present this will return 1
wire parallel_fifo_full;
wire mfp_io0 = (usb_redirection == 2'd2)?parallel_fifo_full:~joy2[4];

// inputs 1,2 and 6 are inputs from serial which have pullups before and inverter
wire  [7:0] mfp_gpio_in = {mfp_io7, 1'b0, !(acsi_irq | fdc_irq), !acia_irq, !blitter_irq, 2'b00, mfp_io0};
wire  [1:0] mfp_timer_in = {de, ste?xsint_delayed:!parallel_fifo_full};
wire  [7:0] mfp_data_out;
wire        mfp_dtack;

wire        mfp_int, mfp_iack = ~mfpiack_n;
assign      mfpint_n = ~mfp_int;

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
	.serial_data_out_available (serial_data_from_mfp_available),
	.serial_strobe_out         (serial_strobe_from_mfp),
	.serial_data_out           (serial_data_from_mfp),
	.serial_status_out         (serial_status_from_mfp),
	.serial_strobe_in          (serial_strobe_to_mfp),
	.serial_data_in            (serial_data_to_mfp),
	.serial_status_in          (serial_status_to_mfp),

	// input signals
	.clk_ext  ( clk_mfp       ),  // 2.457MHz clock
	.t_i      ( mfp_timer_in  ),  // timer a/b inputs
	.i        ( mfp_gpio_in   )   // gpio-in
);

/* ------------------------------------------------------------------------------ */
/* ---------------------------------- IKBD -------------------------------------- */
/* ------------------------------------------------------------------------------ */

wire ikbd_tx, ikbd_rx;

ikbd ikbd (
	.clk(clk_2),
	.res(ikbd_reset),

	.ps2_kbd_clk(ps2_kbd_clk),
	.ps2_kbd_data(ps2_kbd_data),
	.ps2_mouse_clk(ps2_mouse_clk),
	.ps2_mouse_data(ps2_mouse_data),
	.tx(ikbd_tx),
	.rx(ikbd_rx),
	.joystick0({joy0[4], joy0[0], joy0[1], joy0[2], joy0[3]}),
	.joystick1({joy1[4], joy1[0], joy1[1], joy1[2], joy1[3]})
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
	.din      ( cpu_dout[15:8]     ),
	.sel      ( n6850 & ~cpu_a[2]  ),
	.rs       ( cpu_a[1]           ),
	.rw       ( cpu_rw             ),
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

acia midi_acia (
	// cpu interface
	.clk      ( clk_32             ),
	.E        ( cpu_E              ),
	.reset    ( reset              ),
	.din      ( cpu_dout[15:8]     ),
	.sel      ( n6850 & cpu_a[2]   ),
	.rs       ( cpu_a[1]           ),
	.rw       ( cpu_rw             ),
	.dout     ( midi_acia_data_out ),
	.irq      ( midi_acia_irq      ),

	.rx       ( UART_RX            ),
	.tx       ( UART_TX            ),

	// redirected midi interface
	.serial_data_out_available     (midi_data_from_acia_available),
	.serial_strobe_out             (midi_strobe_from_acia),
	.serial_data_out               (midi_data_from_acia)
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------------ PSG ------------------------------------- */
/* ------------------------------------------------------------------------------ */

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
wire [7:0] port_b_in = { ~joy2[0], ~joy2[1], ~joy2[2], ~joy2[3],~joy3[0], ~joy3[1], ~joy3[2], ~joy3[3]};
wire [7:0] port_a_in = { 2'b11, ~joy3[4], 5'b11111 };
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

/* ------------------------------------------------------------------------------ */
/* ------------------------------ Mega STe control ------------------------------ */
/* ------------------------------------------------------------------------------ */

// mega ste cache controller 8 bit interface at $ff8e20 - $ff8e21
// STEroids mode does not have this config, it always runs full throttle
wire       mste_ctrl_sel = !steroids && mste && iodevice && !lds_n && ({cpu_a[15:1], 1'd0} == 16'h8e20);
wire [7:0] mste_ctrl_data_out;
wire       enable_16mhz, enable_cache;

mste_ctrl mste_ctrl (
	// cpu register interface
	.clk      ( clk_32             ),
	.reset    ( reset              ),
	.din      ( cpu_dout[7:0]      ),
	.sel      ( mste_ctrl_sel      ),
	.rw       ( cpu_rw             ),
	.dout     ( mste_ctrl_data_out ),

	.enable_cache ( enable_cache   ),
	.enable_16mhz ( enable_16mhz   )
);

// vme controller 8 bit interface at $ffff8e00 - $ffff8e0f
// (requierd to enable Mega STE cpu speed/cache control)
wire vme_sel = !steroids && mste && iodevice && ({cpu_a[15:4], 4'd0} == 16'h8e00);

/* ------------------------------------------------------------------------------ */
/* ---------------------------------- Blitter ----------------------------------- */
/* ------------------------------------------------------------------------------ */

wire [23:1] blitter_master_addr;
wire blitter_master_write;
wire blitter_master_read;
wire blitter_irq;
wire blitter_br;
wire blitter_bgack;
wire blitter_bg_n;
wire [15:0] blitter_master_data_out;
// blitter 16 bit interface at $ff8a00 - $ff8a3f, STE always has a blitter
wire blitter_sel = (system_ctrl[19] || ste) && iodevice && ~(uds_n && lds_n) && ({cpu_a[15:6], 6'd0} == 16'h8a00);
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

	.br_in       ( ~(mcu_br_n & mcu_bgack_n) ),
	.br_out      ( blitter_br    ),
	.bg          ( ~blitter_bg_n ),
	.irq         ( blitter_irq   ),
	.bgack       ( blitter_bgack ),

	.turbo       ( 0             )
);

/* ------------------------------------------------------------------------------ */
/* ----------------------------- MiST data IO + DMA ----------------------------- */
/* ------------------------------------------------------------------------------ */

wire        dio_data_in_strobe_uio;
wire        dio_data_in_strobe_mist;
wire [15:0] dio_data_in_reg;
wire        dio_data_out_strobe;
wire [15:0] dio_data_out_reg;
wire        dio_dma_ack;
wire  [7:0] dio_dma_status;
wire        dio_dma_nak;
wire  [7:0] dio_status_in;
wire  [3:0] dio_status_index;
wire [23:1] dio_data_addr;
wire        dio_download;

wire [31:0] system_ctrl;

data_io data_io (
	.sck             ( SPI_SCK             ),
	.ss			     ( SPI_SS2             ),
	.sdi   		     ( SPI_DI              ),
	.sdo             ( dio_sdo             ),
	.clk             ( clk_32              ),
	.ctrl_out        ( system_ctrl         ),
	.video_adj       ( ),
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

wire dma_write, dma_read;
wire [15:0] dma_data_out;

wire acsi_irq;

dma dma (
	// system interface
	.clk          ( clk_32        ),
	.clk_en       ( mhz8_en1      ),
	.reset        ( reset         ),

	// cpu interface
	.cpu_din      ( cpu_dout      ),
	.cpu_sel      ( ~fcs_n        ),
	.cpu_a1       ( cpu_a[1]      ),
	.cpu_rw       ( cpu_rw        ),
	.cpu_dout     ( dma_data_out  ),

	// IO controller interface for ACSI
	.dio_data_in_strobe  ( dio_data_in_strobe_mist ),
	.dio_data_in_reg     ( dio_data_in_reg     ),
	.dio_data_out_strobe ( dio_data_out_strobe ),
	.dio_data_out_reg    ( dio_data_out_reg    ),
	.dio_dma_ack         ( dio_dma_ack         ),
	.dio_dma_status      ( dio_dma_status      ),
	.dio_dma_nak         ( dio_dma_nak         ),
	.dio_status_in       ( dio_status_in       ),
	.dio_status_index    ( dio_status_index    ),

	// additional signals for ACSI interface
	.acsi_irq     ( acsi_irq           ),
	.acsi_enable  ( system_ctrl[17:10] ),

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

fdc1772 #(.SECTOR_SIZE_CODE(2'd2),.SECTOR_BASE(1'b1)) fdc1772 (
	.clkcpu         ( clk_32           ), // system cpu clock.
	.clk8m_en       ( mhz8_en1         ),

	// external set signals
	.floppy_drive   ( {2'b11, floppy_sel_exclusive} ),
	.floppy_side    ( floppy_side      ),
	.floppy_reset   ( ~reset           ),

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
	.img_wp         ( system_ctrl[7:6] ), // write protect. latched at img_mounted
	.img_size       ( img_size         ), // size of image in bytes
	.sd_lba         ( sd_lba           ),
	.sd_rd          ( sd_rd            ),
	.sd_wr          ( sd_wr            ),
	.sd_ack         ( sd_ack           ),
	.sd_buff_addr   ( sd_buff_addr     ),
	.sd_dout        ( sd_dout          ),
	.sd_din         ( sd_din           ),
	.sd_dout_strobe ( sd_dout_strobe   ),
	.sd_din_strobe  ( sd_din_strobe    )
);

/* ------------------------------------------------------------------------------ */
/* --------------------------- SDRAM bus multiplexer ---------------------------- */
/* ------------------------------------------------------------------------------ */

// Current blitter implemantation doesn't use the MMU
wire blitter_has_bus = blitter_bgack;

wire cpu_precycle = (bus_cycle == 0);
wire cpu_cycle    = (bus_cycle == 1);
wire viking_cycle = (bus_cycle == 2); // this is the shifter cycle, too

reg ras_n_d;
reg data_wr;
wire ram_oe = ras_n_d & ~ras_n & ram_we_n & |ram_a;
wire ram_we = ras_n_d & ~ras_n & ~ram_we_n;

// TOS/cartridge upload via data_io
reg tos192k = 1'b0;

always @(posedge clk_32) begin
	reg dio_data_in_strobe_uioD;

	ras_n_d <= ras_n;
	data_wr <= 1'b0;
	if (cpu_precycle && mhz8_en1) begin
		dio_data_in_strobe_uioD <= dio_data_in_strobe_uio;
		if (dio_data_in_strobe_uio ^ dio_data_in_strobe_uioD) data_wr <= 1'b1;
	end
	if (dio_download) begin
		if (dio_data_addr[23:18] == 6'b111111) tos192k <= 1'b1;
		else if (dio_data_addr[23:20] == 4'he) tos192k <= 1'b0;
	end
end

// ----------------- RAM address --------------
wire [23:1] sdram_address = (cpu_cycle & dio_download)?dio_data_addr:
                            (cpu_cycle & blitter_has_bus)?blitter_master_addr:
                            (viking_cycle & viking_active & viking_read)?viking_vaddr:ram_a;

wire        ram_en = (MEM512K & ram_a[23:19] == 5'b00000) ||
                     (MEM1M   & ram_a[23:20] == 4'b0000)  ||
                     (MEM2M   & ram_a[23:21] == 3'b000)   ||
                     (MEM4M   & ram_a[23:22] == 2'b00)    ||
                     (MEM8M   & ram_a[23] == 1'b0)        ||
                     (MEM14M  & (~ram_a[23] | ~ram_a[22] | (ram_a[23] & ram_a[22] & ~ram_a[21])));

// ----------------- RAM read -----------------
wire sdram_oe = (cpu_cycle & dio_download)?1'b0:
                (cpu_cycle & blitter_master_read)?1'b1:
                (viking_cycle & viking_active & viking_read)?1'b1:(ram_oe & ram_en);

// ----------------- RAM write -----------------
wire sdram_we = (cpu_cycle & dio_download)?data_wr:(cpu_cycle & blitter_master_write)?1'b1:(ram_we & ram_en);

wire [15:0] ram_data_in = dio_download?dio_data_in_reg:(blitter_has_bus?blitter_master_data_out:ram_din);

// data strobe
wire sdram_uds = (cpu_cycle & (blitter_has_bus | dio_download))?1'b1:ram_uds;
wire sdram_lds = (cpu_cycle & (blitter_has_bus | dio_download))?1'b1:ram_lds;

wire [23:1] rom_a = (!rom2_n & ~tos192k) ? { 4'hE, 2'b00, cpu_a[17:1] } :
                    (!rom2_n &  tos192k) ? { 4'hF, 2'b11, cpu_a[17:1] } : cpu_a;

wire [15:0] ram_data_out;
wire [63:0] ram_data_out64;
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
	.clk_96         ( clk_96                   ),
	.clk_8_en       ( mhz8_en1                 ),
	.init         	( init                     ),

	// cpu/chipset interface
	.din            ( ram_data_in              ),
	.addr           ( { 1'b0, sdram_address }  ),
	.ds             ( { sdram_uds, sdram_lds } ),
	.we             ( sdram_we                 ),
	.oe             ( sdram_oe                 ),
	.dout           ( ram_data_out             ),
	.dout64         ( ram_data_out64           ),

	// ROM access port
	.rom_oe         ( ~rom_n                   ),
	.rom_addr       ( rom_a                    ),
	.rom_dout       ( rom_data_out             )
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------ MiST user IO ---------------------------------- */
/* ------------------------------------------------------------------------------ */

// multiplex spi_do, drive it from user_io if that's selected, drive
// it from data_io if it's selected and leave it open else (also
// to be able to monitor sd card data directly)
wire user_io_sdo, dio_sdo;

assign SPI_DO = (CONF_DATA0 == 1'b0)?user_io_sdo:
	((SPI_SS2 == 1'b0)?dio_sdo:1'bZ);

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
wire [5:0] joy0, joy1, joy2, joy3;

// connection between io controller and ethernet controller
//   mac address transfer io controller -> ethernec
wire [31:0] eth_status;
wire [7:0] eth_mac_byte;
wire eth_mac_strobe, eth_mac_begin;
wire [7:0] eth_tx_read_byte;
wire eth_tx_read_strobe, eth_tx_read_begin;
wire [7:0] eth_rx_write_byte;
wire eth_rx_write_strobe, eth_rx_write_begin;

// ps2 keyboard-mouse emulation
wire ps2_kbd_clk;
wire ps2_kbd_data;
wire ps2_mouse_clk;
wire ps2_mouse_data;

wire [2:0] switches;
wire scandoubler_disable;
wire ypbpr;

// sd-card emulation for 2 images
wire [31:0] sd_lba;
wire  [1:0] sd_rd;
wire  [1:0] sd_wr;
wire        sd_ack;
wire        sd_ack_conf;
wire        sd_conf;
wire        sd_sdhc = 1'b1;
wire  [7:0] sd_dout;
wire        sd_dout_strobe;
wire  [7:0] sd_din;
wire        sd_din_strobe;
wire  [8:0] sd_buff_addr;
wire  [1:0] img_mounted;
wire [31:0] img_size;

//// user io has an extra spi channel outside minimig core ////
user_io user_io(
	.clk_sys                     (clk_32),
	// the spi interface
	.SPI_CLK                     (SPI_SCK),
	.SPI_SS_IO                   (CONF_DATA0),
	.SPI_MISO                    (user_io_sdo),
	.SPI_MOSI                    (SPI_DI),

	// extra joysticks
	.joy0                        (joy0),
	.joy1                        (joy1),
	.joy2                        (joy2),
	.joy3                        (joy3),

	// serial/rs232 interface
	.serial_strobe_out           (serial_strobe_from_mfp),
	.serial_data_out             (serial_data_from_mfp),
	.serial_data_out_available   (serial_data_from_mfp_available),
	.serial_status_out           (serial_status_from_mfp),
	.serial_strobe_in            (serial_strobe_to_mfp),
	.serial_data_in              (serial_data_to_mfp),
	.serial_status_in            (serial_status_to_mfp),

	// parallel interface
	.parallel_strobe_out         (parallel_strobe_out),
	.parallel_data_out           (parallel_data_out),
	.parallel_data_out_available (parallel_data_out_available),

	// midi interface
	.midi_strobe_out             (midi_strobe_from_acia),
	.midi_data_out               (midi_data_from_acia),
	.midi_data_out_available     (midi_data_from_acia_available),

	// ethernet
	.eth_status                  (eth_status),
	.eth_mac_begin               (eth_mac_begin),
	.eth_mac_strobe              (eth_mac_strobe),
	.eth_mac_byte                (eth_mac_byte),
	.eth_tx_read_begin           (eth_tx_read_begin),
	.eth_tx_read_strobe          (eth_tx_read_strobe),
	.eth_tx_read_byte            (eth_tx_read_byte),
	.eth_rx_write_begin          (eth_rx_write_begin),
	.eth_rx_write_strobe         (eth_rx_write_strobe),
	.eth_rx_write_byte           (eth_rx_write_byte),

	// PS2 keyboard data
	.ps2_kbd_clk                 (ps2_kbd_clk),
	.ps2_kbd_data                (ps2_kbd_data),
	// PS2 mouse data
	.ps2_mouse_clk               (ps2_mouse_clk),
	.ps2_mouse_data              (ps2_mouse_data),

	// sd-card IO
	.sd_lba                      (sd_lba        ),
	.sd_rd                       (sd_rd         ),
	.sd_wr                       (sd_wr         ),
	.sd_ack                      (sd_ack        ),
	.sd_ack_conf                 (sd_ack_conf   ),
	.sd_conf                     (sd_conf       ),
	.sd_sdhc                     (sd_sdhc       ),
	.sd_dout                     (sd_dout       ),
	.sd_dout_strobe              (sd_dout_strobe),
	.sd_din                      (sd_din        ),
	.sd_din_strobe               (sd_din_strobe ),
	.sd_buff_addr                (sd_buff_addr  ),
	.img_mounted                 (img_mounted   ),
	.img_size                    (img_size      ),

	// io controller requests to disable vga scandoubler
	.scandoubler_disable         (scandoubler_disable),
	.ypbpr                       (ypbpr),
	.SWITCHES                    (switches ),
	.CORE_TYPE                   (8'ha7)    // mist2 core id
);

endmodule
