/********************************************/
/* Atari ST/STe/Mega STe core MiST toplevel */
/********************************************/

module mist_top ( 
	// clock inputs
	input wire            CLOCK_27,   // 27 MHz
`ifdef USE_CLOCK_50
	input wire            CLOCK_50,   // 50 MHz
`endif
	// LED outputs
	output wire           LED,        // LED Yellow
	// UART
	output wire           UART_TX,    // UART Transmitter (MIDI out)
	input wire            UART_RX,    // UART Receiver (MIDI in)
`ifdef SIDI128_EXPANSION
	input wire            UART_CTS,
	output wire           UART_RTS,
	inout wire            EXP7,
	inout wire            MOTOR_CTRL,
`endif
`ifdef USE_MIDI_PINS
	output wire           MIDI_OUT,
	input wire            MIDI_IN,
`endif
	// VGA
	output wire           VGA_HS,     // VGA H_SYNC
	output wire           VGA_VS,     // VGA V_SYNC
	output wire  [VGA_BITS-1:0] VGA_R,      // VGA Red[5:0]
	output wire  [VGA_BITS-1:0] VGA_G,      // VGA Green[5:0]
	output wire  [VGA_BITS-1:0] VGA_B,      // VGA Blue[5:0]
`ifdef USE_HDMI
	output                HDMI_RST,
	output          [7:0] HDMI_R,
	output          [7:0] HDMI_G,
	output          [7:0] HDMI_B,
	output                HDMI_HS,
	output                HDMI_VS,
	output                HDMI_PCLK,
	output                HDMI_DE,
	inout                 HDMI_SDA,
	inout                 HDMI_SCL,
	input                 HDMI_INT,
`endif
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
`ifdef DUAL_SDRAM
	inout wire  [ 16-1:0] SDRAM2_DQ,  // SDRAM Data bus 16 Bits
	output wire [ 13-1:0] SDRAM2_A,   // SDRAM Address bus 13 Bits
	output wire           SDRAM2_DQML,// SDRAM Low-byte Data Mask
	output wire           SDRAM2_DQMH,// SDRAM High-byte Data Mask
	output wire           SDRAM2_nWE, // SDRAM Write Enable
	output wire           SDRAM2_nCAS,// SDRAM Column Address Strobe
	output wire           SDRAM2_nRAS,// SDRAM Row Address Strobe
	output wire           SDRAM2_nCS, // SDRAM Chip Select
	output wire  [ 2-1:0] SDRAM2_BA,  // SDRAM Bank Address
	output wire           SDRAM2_CLK, // SDRAM Clock
	output wire           SDRAM2_CKE, // SDRAM Clock Enable
`endif
	// AUDIO
	output wire           AUDIO_L,    // sigma-delta DAC output left
	output wire           AUDIO_R,    // sigma-delta DAC output right
`ifdef I2S_AUDIO
	output wire           I2S_BCK,
	output wire           I2S_LRCK,
	output wire           I2S_DATA,
`endif
`ifdef I2S_AUDIO_HDMI
	output                HDMI_MCLK,
	output                HDMI_BCK,
	output                HDMI_LRCK,
	output                HDMI_SDATA,
`endif
`ifdef SPDIF_AUDIO
	output                SPDIF,
`endif
	// SPI
	inout wire            SPI_DO,
	input wire            SPI_DI,
	input wire            SPI_SCK,
	input wire            SPI_SS2,    // fpga
	input wire            SPI_SS3,    // OSD
`ifndef NO_DIRECT_UPLOAD
	input wire            SPI_SS4,    // "sniff" mode
`endif
	input wire            CONF_DATA0  // SPI_SS for user_io
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------- System settings ------------------------------ */
/* ------------------------------------------------------------------------------ */
`ifdef NO_DIRECT_UPLOAD
wire SPI_SS4 = 1;
`endif

`ifdef VGA_8BIT
localparam VGA_BITS = 8;
`else
localparam VGA_BITS = 6;
`endif

`ifdef USE_HDMI
localparam bit HDMI = 1;
assign HDMI_RST = 1'b1;
`else
localparam bit HDMI = 0;
`endif

`ifdef NO_TG68K
localparam TG68K_ENABLE = 1'b0;
`else
localparam TG68K_ENABLE = 1'b1;
`endif

// remove this if the 2nd chip is actually used
`ifdef DUAL_SDRAM
assign SDRAM2_A = 13'hZZZZ;
assign SDRAM2_BA = 0;
assign SDRAM2_DQML = 1;
assign SDRAM2_DQMH = 1;
assign SDRAM2_CKE = 0;
assign SDRAM2_CLK = 0;
assign SDRAM2_nCS = 1;
assign SDRAM2_DQ = 16'hZZZZ;
assign SDRAM2_nCAS = 1;
assign SDRAM2_nRAS = 1;
assign SDRAM2_nWE = 1;
`endif

wire reset = system_ctrl[0];
wire [1:0] fdc_wp = system_ctrl[7:6];
// usb target port on io controller is used for redirection of
// 0=nothing 1=rs232 2=printer 3=midi
wire [1:0] usb_redirection = system_ctrl[27:26];
wire blend = system_ctrl[29];

/* ------------------------------------------------------------------------------ */
/* ------------------------------ Clock generation ------------------------------ */
/* ------------------------------------------------------------------------------ */

wire pll_locked;
wire clk_2;
wire clk_32;
wire clk_96;
wire clk_128;

// 32.084 MHz base clock
wire mainclock;
clock32 clock32 (
`ifdef USE_CLOCK_50
	.inclk0     ( CLOCK_50  ),
`else
	.inclk0     ( CLOCK_27  ),
`endif
	.c0         ( mainclock )
);

clock clock (
  .inclk0       (mainclock  ), // input clock (32.084MHz)
  .c0           (clk_96     ), // output clock c0 (96MHz)
  .c1           (clk_32     ), // output clock c1 (32MHz)
  .c2           (clk_128    ), // output clock c2 (128MHz)
  .c3           (clk_2      ), // output clock c3 (2MHz)
  .locked       (pll_locked )  // pll locked output
);

assign SDRAM_CLK = clk_96;
assign SDRAM_CKE = 1'b1;

// generate 2.4576MHz MFP clock
reg [31:0] clk_cnt_mfp;
reg        clk_mfp_en;

localparam SYSTEM_CLOCK = 32'd32_084_988;
localparam MFP_CLOCK    =  32'd2_457_600;

always @(posedge clk_32) begin
	if(reset) begin
		clk_cnt_mfp <= 32'd0;
		clk_mfp_en <= 1'b0;
	end else begin
		clk_mfp_en <= 1'b0;

		if(clk_cnt_mfp < SYSTEM_CLOCK)
			clk_cnt_mfp <= clk_cnt_mfp + MFP_CLOCK;
		else begin
			clk_cnt_mfp <= clk_cnt_mfp - SYSTEM_CLOCK + MFP_CLOCK;
			clk_mfp_en <= 1'b1;
		end
	end
end
/* ------------------------------------------------------------------------------ */
/* -------------------------------- Video output -------------------------------- */
/* ------------------------------------------------------------------------------ */

wire        video_clk;
wire        viking_active;
wire        viking_hs, viking_vs;
wire        viking_hb, viking_vb;
wire  [3:0] viking_r, viking_g, viking_b;
wire        monomode;
wire  [3:0] r, g, b;
wire        hsync_n, vsync_n;
wire        hblank_n, vblank_n;
wire        blank_n;

wire  [1:0] scanlines = system_ctrl[21:20];

vidclkcntrl vidclkcntrl (
	.clkselect ( viking_active   ),
	.inclk0x   ( clk_32          ),
	.inclk1x   ( clk_128         ),
	.outclk    ( video_clk       )
);

wire [3:0] stvid_r   = viking_active?viking_r:(blank_n | monomode) ? r : 4'h0;
wire [3:0] stvid_g   = viking_active?viking_g:(blank_n | monomode) ? g : 4'h0;
wire [3:0] stvid_b   = viking_active?viking_b:(blank_n | monomode) ? b : 4'h0;
wire       stvid_hs  = viking_active?viking_hs:hsync_n;
wire       stvid_vs  = viking_active?viking_vs:vsync_n;
wire       stvid_hb  = viking_active?viking_hb:!hblank_n;
wire       stvid_vb  = viking_active?viking_vb:!vblank_n;

mist_video #(.OSD_COLOR(3'b010), .COLOR_DEPTH(4), .SD_HCNT_WIDTH(10), .OSD_X_OFFSET(10'd10), .OUT_COLOR_DEPTH(VGA_BITS)) mist_video(
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
	.blend      ( blend & ~monomode & ~viking_active ),
	.scandoubler_disable( scandoubler_disable | monomode | viking_active ),
	.no_csync   ( monomode | viking_active | no_csync ),
	.scanlines  ( scanlines ),
	.ypbpr      ( ypbpr )
);

`ifdef USE_HDMI
i2c_master #(32_000_000) i2c_master (
	.CLK         (clk_32),
	.I2C_START   (i2c_start),
	.I2C_READ    (i2c_read),
	.I2C_ADDR    (i2c_addr),
	.I2C_SUBADDR (i2c_subaddr),
	.I2C_WDATA   (i2c_dout),
	.I2C_RDATA   (i2c_din),
	.I2C_END     (i2c_end),
	.I2C_ACK     (i2c_ack),

	//I2C bus
	.I2C_SCL     (HDMI_SCL),
	.I2C_SDA     (HDMI_SDA)
);

mist_video #(.OSD_COLOR(3'b010), .COLOR_DEPTH(4), .SD_HCNT_WIDTH(10), .OSD_X_OFFSET(10'd10), .OUT_COLOR_DEPTH(8), .USE_BLANKS(1), .VIDEO_CLEANER(1)) hdmi_video(
	.clk_sys    ( video_clk ),
	.SPI_SCK    ( SPI_SCK ),
	.SPI_SS3    ( SPI_SS3 ),
	.SPI_DI     ( SPI_DI ),
	.R          ( stvid_r ),
	.G          ( stvid_g ),
	.B          ( stvid_b ),
	.HSync      ( stvid_hs ),
	.VSync      ( stvid_vs ),
	.HBlank     ( stvid_hb ),
	.VBlank     ( stvid_vb ),
	.VGA_R      ( HDMI_R ),
	.VGA_G      ( HDMI_G ),
	.VGA_B      ( HDMI_B ),
	.VGA_VS     ( HDMI_VS ),
	.VGA_HS     ( HDMI_HS ),
	.VGA_DE     ( HDMI_DE ),
	.ce_divider ( 3'd1 ),
	.rotate     ( 2'b00 ),
	.blend      ( blend & ~monomode & ~viking_active ),
	.scandoubler_disable( monomode | viking_active ),
	.no_csync   ( 1'b1 ),
	.scanlines  ( scanlines ),
	.ypbpr      ( 1'b0 )
);

assign HDMI_PCLK = video_clk;

`endif

`ifdef I2S_AUDIO
i2s i2s (
	.reset(1'b0),
	.clk(clk_32),
	.clk_rate(32'd32_084_999),

	.sclk(I2S_BCK),
	.lrclk(I2S_LRCK),
	.sdata(I2S_DATA),

	.left_chan({audio_mix_l[14], audio_mix_l}),
	.right_chan({audio_mix_r[14], audio_mix_r})
);
`ifdef I2S_AUDIO_HDMI
assign HDMI_MCLK = 0;
always @(posedge clk_32) begin
	HDMI_BCK <= I2S_BCK;
	HDMI_LRCK <= I2S_LRCK;
	HDMI_SDATA <= I2S_DATA;
end
`endif
`endif

`ifdef SPDIF_AUDIO
spdif spdif
(
	.clk_i(clk_32),
	.rst_i(1'b0),
	.clk_rate_i(32'd32_084_999),
	.spdif_o(SPDIF),
	.sample_i({audio_mix_r[14], audio_mix_r, audio_mix_l[14], audio_mix_l})
);
`endif

/* ------------------------------------------------------------------------------ */
/* ------------------------------- Sigma-delta DAC ------------------------------ */
/* ------------------------------------------------------------------------------ */

wire [14:0] audio_mix_l, audio_mix_r;

sigma_delta_dac sigma_delta_dac (
	.clk      ( clk_32      ),      // bus clock
	.ldatasum ( audio_mix_l ),      // left channel data
	.rdatasum ( audio_mix_r ),      // right channel data
	.left     ( AUDIO_L     ),      // left bitstream output
	.right    ( AUDIO_R     )       // right bitsteam output
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------- MIDO output FIFO ----------------------------- */
/* ------------------------------------------------------------------------------ */
// filled by the CPU when writing to the acia data register
// emptied by the io controller when reading via SPI
// This happens in parallel to the real serial generation, so 
// physical and USB serial can be used at the same time

wire        midi_out_strobe;
wire  [7:0] midi_out;

io_fifo midi_out_fifo (
	.reset        ( reset ),

	.in_clk       ( clk_32 ),
	.in           ( midi_out ),
	.in_strobe    ( 1'b0 ),
	.in_enable    ( midi_out_strobe ),  // acia data write

	.out_clk      ( clk_32 ),
	.out          ( midi_data_from_acia ),
	.out_strobe   ( midi_strobe_from_acia ),
	.out_enable   ( 1'b0 ),

	.data_available ( midi_data_from_acia_available )
);

/* ------------------------------------------------------------------------------ */
/* -------------------------- Parallel port output FIFO ------------------------- */
/* ------------------------------------------------------------------------------ */
wire        parallel_out_strobe;
wire  [7:0] parallel_out;
wire        parallel_fifo_full;

// The printer busy signal is pulled down when the printer cannot accept further data
// if no printer redirection is being used this is wired to the extra joystick ports provided
// by the "gauntlet2 adapter". If no extra joystick ports are present, the busy signal will unconnected,
// and it's pulled up internally.

wire parallel_printer_busy = (usb_redirection == 2'd2)?parallel_fifo_full:joy2[4];

io_fifo #(.DEPTH(4)) parallel_out_fifo (
	.reset          ( reset ),

	.in_clk         ( clk_32 ),
	.in             ( parallel_out ),
	.in_strobe      ( parallel_out_strobe ),
	.in_enable      ( 1'b0 ),

	.out_clk        ( clk_32 ),
	.out            ( parallel_data_out ),
	.out_strobe     ( parallel_strobe_out ),
	.out_enable     ( 1'b0 ),

	.full           ( parallel_fifo_full ),
	.data_available ( parallel_data_out_available )
);

// extra joysticks are wired to the printer port
// using the "gauntlet2 interface", fire of
// joystick 0 is connected to the mfp I0 (busy)
wire [7:0] parallel_in = { ~joy2[0], ~joy2[1], ~joy2[2], ~joy2[3],~joy3[0], ~joy3[1], ~joy3[2], ~joy3[3] };
wire       parallel_in_strobe = ~joy3[4];

/* ------------------------------------------------------------------------------ */
/* -------------------------------- MiST data IO -------------------------------- */
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

wire        spi_din = SPI_SS4 ? SPI_DI : SPI_DO;

data_io data_io (
	.sck             ( SPI_SCK             ),
	.ss              ( SPI_SS2             ),
	.ss_sd           ( SPI_SS4             ),
	.sdi             ( spi_din             ),
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
wire [63:0] serial_status_from_mfp;

// connection to transfer parallel data from psg to io controller
wire [7:0] parallel_data_out;
wire parallel_strobe_out;
wire parallel_data_out_available;

// extra joystick interface
wire [15:0] joy0, joy1, joy2, joy3;

// RTC
wire [63:0] rtc;

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
wire no_csync;

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
wire  [8:0] sd_buff_addr;
wire  [1:0] img_mounted;
wire [31:0] img_size;

`ifdef USE_HDMI
wire        i2c_start;
wire        i2c_read;
wire  [6:0] i2c_addr;
wire  [7:0] i2c_subaddr;
wire  [7:0] i2c_dout;
wire  [7:0] i2c_din;
wire        i2c_ack;
wire        i2c_end;
`endif

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
`ifdef USE_HDMI
	.i2c_start                   (i2c_start     ),
	.i2c_read                    (i2c_read      ),
	.i2c_addr                    (i2c_addr      ),
	.i2c_subaddr                 (i2c_subaddr   ),
	.i2c_dout                    (i2c_dout      ),
	.i2c_din                     (i2c_din       ),
	.i2c_ack                     (i2c_ack       ),
	.i2c_end                     (i2c_end       ),
	.hdmi_hiclk                  (viking_active ),
`endif
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
	.sd_buff_addr                (sd_buff_addr  ),
	.img_mounted                 (img_mounted   ),
	.img_size                    (img_size      ),

	// io controller requests to disable vga scandoubler
	.scandoubler_disable         (scandoubler_disable),
	.ypbpr                       (ypbpr),
	.no_csync                    (no_csync),
	.rtc                         (rtc),
	.SWITCHES                    (switches ),
	.CORE_TYPE                   (8'ha7)    // mist2 core id
);

/* ------------------------------------------------------------------------------ */
/* ------------------------------ The ATARI ST ---------------------------------- */
/* ------------------------------------------------------------------------------ */
atarist_sdram #(TG68K_ENABLE, 1'b1) atarist(
	// System clocks / reset / settings
	.clk_96              ( clk_96 ),
	.clk_32              ( clk_32 ),
	.clk_128             ( clk_128 ),
	.clk_2               ( clk_2 ),
	.clk_mfp             ( clk_mfp_en ),
	.porb                ( pll_locked ),
	.system_ctrl         ( system_ctrl ),

	// Video output
	.r                   ( r ),
	.g                   ( g ),
	.b                   ( b ),
	.hsync_n             ( hsync_n ),
	.vsync_n             ( vsync_n ),
	.monomode            ( monomode ),
	.blank_n             ( blank_n ),
	.hblank_n            ( hblank_n ),
	.vblank_n            ( vblank_n ),

	.viking_active       ( viking_active ),
	.viking_r            ( viking_r ),
	.viking_g            ( viking_g ),
	.viking_b            ( viking_b ),
	.viking_hs           ( viking_hs ),
	.viking_vs           ( viking_vs ),
	.viking_hb           ( viking_hb ),
	.viking_vb           ( viking_vb ),

	// Sound output
	.audio_mix_l         ( audio_mix_l ),
	.audio_mix_r         ( audio_mix_r ),

	// MIDI OUT (parallel data)
	.midi_out_strobe     ( midi_out_strobe ),
	.midi_out            ( midi_out ),

	// MIDI UART
`ifdef USE_MIDI_PINS
	.serial_redirect     ( usb_redirection == 1 ), // rs232 redirection to USB
	.midi_rx             ( MIDI_IN ),
	.midi_tx             ( MIDI_OUT ),
	.uart_rx             ( UART_RX ),
	.uart_tx             ( UART_TX ),
`else
	.serial_redirect     ( 1'b1 ),
	.midi_rx             ( UART_RX ),
	.midi_tx             ( UART_TX ),
`endif
`ifdef SIDI128_EXPANSION
	.uart_rtsb           ( UART_RTS ),
	.uart_ctsb           ( UART_CTS ),
`else
	.uart_rtsb           ( ),
	.uart_ctsb           ( 1'b0 ),
`endif
	// Parallel port IN-OUT
	.parallel_in_strobe  ( parallel_in_strobe ),
	.parallel_in         ( parallel_in ),
	.parallel_out_strobe ( parallel_out_strobe ),
	.parallel_out        ( parallel_out ),
	.parallel_printer_busy ( parallel_printer_busy ),

	// Serial port IN/OUT
	.serial_data_out_available (serial_data_from_mfp_available),
	.serial_strobe_out   ( serial_strobe_from_mfp ),
	.serial_data_out     ( serial_data_from_mfp ),
	.serial_status_out   ( serial_status_from_mfp ),
	.serial_strobe_in    ( serial_strobe_to_mfp ),
	.serial_data_in      ( serial_data_to_mfp ),

	// ROM/CART download / ACSI
	.data_in_strobe_rom  ( dio_data_in_strobe_uio ),
	.data_in_strobe_acsi ( dio_data_in_strobe_mist ),
	.data_in_reg         ( dio_data_in_reg ),
	.data_addr           ( dio_data_addr ),
	.data_download       ( dio_download ),

	.data_out_strobe     ( dio_data_out_strobe ),
	.data_out_reg        ( dio_data_out_reg ),
	.dma_ack             ( dio_dma_ack ),
	.dma_status          ( dio_dma_status ),
	.dma_nak             ( dio_dma_nak ),
	.dma_status_in       ( dio_status_in ),
	.dma_status_index    ( dio_status_index ),

	// FDC
	.img_mounted         ( img_mounted      ), // signaling that new image has been mounted
	.img_wp              ( fdc_wp           ), // write protect
	.img_size            ( img_size         ), // size of image in bytes
	.sd_lba              ( sd_lba           ),
	.sd_rd               ( sd_rd            ),
	.sd_wr               ( sd_wr            ),
	.sd_ack              ( sd_ack           ),
	.sd_buff_addr        ( sd_buff_addr     ),
	.sd_dout             ( sd_dout          ),
	.sd_din              ( sd_din           ),
	.sd_dout_strobe      ( sd_dout_strobe   ),
	.LED                 ( LED              ),

	// Ethernet
	.eth_status          ( eth_status ),
	.eth_mac_begin       ( eth_mac_begin ),
	.eth_mac_strobe      ( eth_mac_strobe ),
	.eth_mac_byte        ( eth_mac_byte ),
	.eth_tx_read_begin   ( eth_tx_read_begin ),
	.eth_tx_read_strobe  ( eth_tx_read_strobe ),
	.eth_tx_read_byte    ( eth_tx_read_byte ),
	.eth_rx_write_begin  ( eth_rx_write_begin ),
	.eth_rx_write_strobe ( eth_rx_write_strobe ),
	.eth_rx_write_byte   ( eth_rx_write_byte ),

	// PS2 keyboard data
	.ps2_kbd_clk         ( ps2_kbd_clk ),
	.ps2_kbd_data        ( ps2_kbd_data ),

	// PS2 mouse data
	.ps2_mouse_clk       ( ps2_mouse_clk ),
	.ps2_mouse_data      ( ps2_mouse_data ),

	// joysticks
	.joy0                ( joy0 ),
	.joy1                ( joy1 ),

	// RTC
	.rtc                 ( rtc ),

	// SDRAM pins
	.SDRAM_DQ            ( SDRAM_DQ ),   // SDRAM Data bus 16 Bits
	.SDRAM_A             ( SDRAM_A  ),   // SDRAM Address bus 13 Bits
	.SDRAM_DQML          ( SDRAM_DQML ), // SDRAM Low-byte Data Mask
	.SDRAM_DQMH          ( SDRAM_DQMH ), // SDRAM High-byte Data Mask
	.SDRAM_nWE           ( SDRAM_nWE  ), // SDRAM Write Enable
	.SDRAM_nCAS          ( SDRAM_nCAS ), // SDRAM Column Address Strobe
	.SDRAM_nRAS          ( SDRAM_nRAS ), // SDRAM Row Address Strobe
	.SDRAM_BA            ( SDRAM_BA   ), // SDRAM Bank Address
	.SDRAM_nCS           ( SDRAM_nCS  )  // SDRAM Chip Select
);

`ifdef SIDI128_EXPANSION
	assign EXP7 = 1'bZ;
	assign MOTOR_CTRL = 1'bZ;
`endif

endmodule
