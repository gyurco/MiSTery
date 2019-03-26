// dma.v
//
// Atari ST dma engine for the MIST baord
// http://code.google.com/p/mist-board/
//
// This file implements a SPI client which can write data
// into any memory region. This is used to upload rom
// images as well as implementation the DMA functionality
// of the Atari ST DMA controller.
//
// This also implements the video adjustment. This has nothing
// to do with dma and should happen in user_io instead.
// But now it's here and moving it is not worth the effort.
//
// Copyright (c) 2014 Till Harbaum <till@harbaum.org>
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

// TODO:
// - Allow DMA transfers to ROM only for IO controller initiated transfers

// ##############DMA/WD1772 Disk controller                           ###########
// -------+-----+-----------------------------------------------------+----------
// $FF8600|     |Reserved                                             |
// $FF8602|     |Reserved                                             |
// $FF8604|word |FDC access/sector count                              |R/W
// $FF8606|word |DMA mode/status                             BIT 2 1 0|R
//        |     |Condition of FDC DATA REQUEST signal -----------' | ||
//        |     |0 - sector count null,1 - not null ---------------' ||
//        |     |0 - no error, 1 - DMA error ------------------------'|
// $FF8606|word |DMA mode/status                 BIT 8 7 6 . 4 3 2 1 .|W
//        |     |0 - read FDC/HDC,1 - write ---------' | | | | | | |  |
//        |     |0 - HDC access,1 - FDC access --------' | | | | | |  |
//        |     |0 - DMA on,1 - no DMA ------------------' | | | | |  |
//        |     |Reserved ---------------------------------' | | | |  |
//        |     |0 - FDC reg,1 - sector count reg -----------' | | |  |
//        |     |0 - FDC access,1 - HDC access ----------------' | |  |
//        |     |0 - pin A1 low, 1 - pin A1 high ----------------' |  |
//        |     |0 - pin A0 low, 1 - pin A0 high ------------------'  |
// $FF8609|byte |DMA base and counter (High byte)                     |R/W
// $FF860B|byte |DMA base and counter (Mid byte)                      |R/W
// $FF860D|byte |DMA base and counter (Low byte)                      |R/W
//        |     |Note: write address from low toward high byte        |
	 
module dma (
	    // clocks and system interface
	input 		  clk,
	input         clk_32,
	input         clk_en,
	input 		  reset,
	input [1:0] 	  bus_cycle,
	input 		  turbo,
	output 		  irq,

	// cpu interface
	input      [15:0] cpu_din,
	input 		      cpu_sel,
	input      [ 2:0] cpu_addr,
	input 		      cpu_uds,
	input 		      cpu_lds,
	input 		      cpu_rw,
	output reg [15:0] cpu_dout,

	input             dio_addr_strobe,
	input      [31:0] dio_addr_reg,

	input             dio_data_in_strobe,
	input      [15:0] dio_data_in_reg,

	input             dio_data_out_strobe,
	output     [15:0] dio_data_out_reg,

	input             dio_dma_ack,
	input       [7:0] dio_dma_status,

	input             dio_dma_nak,
	output      [7:0] dio_status_in,
	input       [4:0] dio_status_index,

	// additional fdc control signals provided by PSG and OSD
	input 		  fdc_wr_prot,
	input 		  drv_side,
	input [1:0] 	  drv_sel,
	// additional acsi control signals provided by OSD
	input [7:0]       acsi_enable,

	// ram interface for dma engine
	output 		  ram_br,
	output reg 	  ram_read,
	output reg 	  ram_write,
	output [22:0] 	  ram_addr,
	output [15:0] 	  ram_dout,
	input [15:0] 	  ram_din
);

assign irq = fdc_irq || acsi_irq;

// for debug: count irqs
reg [7:0] fdc_irq_count;
always @(posedge clk_32 or posedge reset) begin
	reg fdc_irqD;
	if(reset) fdc_irq_count <= 8'd0;
	else begin
		fdc_irqD <= fdc_irq;
		if (~fdc_irqD & fdc_irq) fdc_irq_count <= fdc_irq_count + 8'd1;
	end
end

reg [7:0] acsi_irq_count;
always @(posedge clk_32 or posedge reset) begin
	reg acsi_irqD;
	if(reset) acsi_irq_count <= 8'd0;
	else begin
		acsi_irqD <= acsi_irq;
		if (~acsi_irqD & acsi_irq) acsi_irq_count <= acsi_irq_count + 8'd1;
	end
end

reg cpu_selD;
always @(posedge clk_32) if (clk_en) cpu_selD <= cpu_sel;
wire cpu_req = ~cpu_selD & cpu_sel;

// dma sector count and mode registers
reg [7:0]  dma_scnt;   
reg [15:0] dma_mode;

// ============= FDC submodule ============   

// select signal for the fdc controller registers   
wire    fdc_reg_sel = cpu_sel && !cpu_lds && (cpu_addr == 3'h2) && (dma_mode[4:3] == 2'b00);
wire    fdc_irq;   
wire [7:0] fdc_status_byte;
wire [7:0] fdc_dout;
   
fdc fdc(
	 .clk         ( clk_32                ),
	 .clk_en      ( clk_en                ),
	 .reset       ( reset                 ),
	 
	 .irq         ( fdc_irq               ),

	 // external floppy control signals
	 .drv_sel     ( drv_sel               ),
	 .drv_side    ( drv_side              ),
	 .wr_prot     ( fdc_wr_prot				),

	 // signals from/to io controller
	 .dma_ack     ( io_dma_ack            ),
	 .status_sel  ( bcnt-4'd4             ),
	 .status_byte ( fdc_status_byte       ),
	 
	 // cpu interfaces, passed trough dma in st
	 .cpu_sel     ( fdc_reg_sel           ),
	 .cpu_addr    ( dma_mode[2:1]         ),
	 .cpu_rw      ( cpu_rw                ),
	 .cpu_din     ( cpu_din[7:0]          ),
	 .cpu_dout    ( fdc_dout              )
);
   
// ============= ACSI submodule ============   

// select signal for the acsi controller access (write only, status comes from io controller)
wire    acsi_reg_sel = cpu_sel && !cpu_lds && (cpu_addr == 3'h2) && (dma_mode[4:3] == 2'b01);
wire    acsi_irq;   
wire [7:0] acsi_status_byte;
wire [7:0] acsi_dout;
 
acsi acsi(
	 .clk         ( clk_32                ),
	 .clk_en      ( clk_en                ),
	 .reset       ( reset                 ),
	 
	 .irq         ( acsi_irq              ),

	  // acsi target enable
	 .enable      ( acsi_enable           ),

	 // signals from/to io controller
	 .dma_ack     ( io_dma_ack            ),
	 .dma_nak     ( io_dma_nak            ),
	 .dma_status  ( dio_dma_status        ),
	  
	 .status_sel  ( bcnt-4'd9             ),
	 .status_byte ( acsi_status_byte      ),
	 
	 // cpu interface, passed through dma in st
	 .cpu_sel     ( acsi_reg_sel          ),
	 .cpu_addr    ( dma_mode[2:1]         ),
	 .cpu_rw      ( cpu_rw                ),
	 .cpu_din     ( cpu_din[7:0]          ),
	 .cpu_dout    ( acsi_dout             )
);

// ============= CPU read interface ============
always @(cpu_sel, cpu_rw, cpu_addr, dma_mode, dma_addr, dma_scnt, fdc_dout, acsi_dout) begin
   cpu_dout = 16'h0000;

   if(cpu_sel && cpu_rw) begin
      
      // register $ff8604
      if(cpu_addr == 3'h2) begin
         if(dma_mode[4] == 1'b0) begin
            // controller access register
            if(dma_mode[3] == 1'b0)
					cpu_dout = { 8'h00, fdc_dout };
				else
					cpu_dout = { 8'h00, acsi_dout };
         end else
           cpu_dout = { 8'h00, dma_scnt };  // sector count register
      end
      
      // DMA status register $ff8606
      // bit 0 = 1: DMA_OK, bit 2 = state of FDC DRQ
      if(cpu_addr == 3'h3) cpu_dout = { 14'd0, dma_scnt != 0, 1'b1 };
      
      // dma address read back at $ff8609-$ff860d
      if(cpu_addr == 3'h4) cpu_dout = { 8'h00, dma_addr[22:15]     };
      if(cpu_addr == 3'h5) cpu_dout = { 8'h00, dma_addr[14:7]      };
      if(cpu_addr == 3'h6) cpu_dout = { 8'h00, dma_addr[6:0], 1'b0 };
   end
end

// ============= CPU write interface ============
// flags indicating the cpu is writing something. valid on rising edge
reg cpu_address_write_strobe;  
reg cpu_scnt_write_strobe;  
reg cpu_mode_write_strobe;  
reg cpu_dma_mode_direction_toggle;
   
always @(posedge clk_32) begin
   if(reset) begin
      cpu_address_write_strobe <= 1'b0;      
      cpu_scnt_write_strobe <= 1'b0;      
      cpu_mode_write_strobe <= 1'b0;
      cpu_dma_mode_direction_toggle <= 1'b0;
   end else begin
      cpu_address_write_strobe <= 1'b0;      
      cpu_scnt_write_strobe <= 1'b0;      
      cpu_mode_write_strobe <= 1'b0;
      cpu_dma_mode_direction_toggle <= 1'b0;

      // cpu writes ...
      if(clk_en && cpu_req && !cpu_rw && !cpu_lds) begin

	 // ... sector count register
	 if((cpu_addr == 3'h2) && (dma_mode[4] == 1'b1))
	   cpu_scnt_write_strobe <= 1'b1;
	 
	 // ... dma mode register
	 if(cpu_addr == 3'h3) begin
	   dma_mode <= cpu_din;
	    cpu_mode_write_strobe <= 1'b1;

	   // check if cpu toggles direction bit (bit 8)
	   if(dma_mode[8] != cpu_din[8])
	     cpu_dma_mode_direction_toggle <= 1'b1;
	 end
	 
	 // ... dma address
         if((cpu_addr == 3'h4) || (cpu_addr == 3'h5) || (cpu_addr == 3'h6))
	   // trigger address engine latch
	   cpu_address_write_strobe <= 1'b1;      
      end
   end
end

// ======================== BUS cycle handler ===================

// specify which bus cycles to use
wire cycle_advance   = (bus_cycle == 2'd0) || (turbo && (bus_cycle == 2'd2));
wire cycle_io        = (bus_cycle == 2'd1) || (turbo && (bus_cycle == 2'd3));

// latch bus cycle information to use at the end of the cycle (posedge clk)
reg cycle_advance_L;
always @(negedge clk) begin
	cycle_advance_L <= cycle_advance;
end

// =======================================================================
// ============================= DMA FIFO ================================
// =======================================================================

// 32 byte dma fifo (actually a 16 word fifo)
reg [15:0] fifo [15:0];
reg [3:0] fifo_wptr;         // word pointers
reg [3:0] fifo_rptr;

// stop reading from fifo if it is empty
wire fifo_empty     = fifo_wptr == fifo_rptr;
// fifo is considered full if 8 words (16 bytes) are present
wire fifo_full      = (fifo_wptr - fifo_rptr) > 4'd7;
// Reset fifo via the dma mode direction bit toggling or when
// IO controller sets address
wire fifo_reset = cpu_dma_mode_direction_toggle || io_addr_strobe;

reg fifo_read_in_progress, fifo_write_in_progress;
assign ram_br = fifo_read_in_progress || fifo_write_in_progress || ioc_br_clk; 

reg ioc_br_clk;
always @(negedge clk)
   if(cycle_advance) 
		ioc_br_clk <= ioc_br;

// ============= FIFO WRITE ENGINE ==================

// start condition for fifo write
wire fifo_write_start = dma_in_progress && dma_direction_out && 
     !fifo_write_in_progress && !fifo_full;

// state machine for DMA ram read access
always @(posedge clk_32 or posedge fifo_reset) begin
	if(fifo_reset == 1'b1) begin
		fifo_write_in_progress <= 1'b0;
	end else begin
		if(cycle_advance) begin
			// start dma read engine if 8 more words can be stored
			if(fifo_write_start) fifo_write_in_progress <= 1'b1;
			else if (fifo_full) fifo_write_in_progress <= 1'b0;
		end
	end
end
   
// ram control signals need to be stable over the whole 8 Mhz cycle
always @(posedge clk)
   ram_read <= (cycle_advance_L && fifo_write_in_progress);

wire ram_read_strobe = ~ram_readD & ram_read;
wire ram_read_done   = ram_readD & ~ram_read;
reg ram_readD;
always @(posedge clk_32)
	ram_readD <= ram_read;

wire [15:0] fifo_data_in = dma_direction_out?ram_din:dio_data_in_reg;
wire fifo_data_in_strobe = dma_direction_out?ram_read_done:io_data_in_strobe;

// write to fifo on rising edge of fifo_data_in_strobe
always @(posedge clk_32 or posedge fifo_reset) begin
   if(fifo_reset == 1'b1)
     fifo_wptr <= 4'd0;
   else if (fifo_data_in_strobe) begin
      fifo[fifo_wptr] <= fifo_data_in;
      fifo_wptr <= fifo_wptr + 4'd1;
   end
end

// ============= FIFO READ ENGINE ==================

// start condition for fifo read
wire fifo_read_start = dma_in_progress && !dma_direction_out && 
     !fifo_read_in_progress && fifo_full;

// state machine for DMA ram write access
always @(posedge clk_32 or posedge fifo_reset) begin
	if(fifo_reset == 1'b1) begin
		// not reading from fifo, not writing into ram
		fifo_read_in_progress <= 1'b0;
	end else begin
		// start dma read engine if 8 more words can be stored
		if(cycle_advance) begin
			if (fifo_read_start) fifo_read_in_progress <= 1'b1;
			if (fifo_empty) fifo_read_in_progress <= 1'b0;
		end
	end
end

// ram control signals need to be stable over the whole 8 Mhz cycle
always @(posedge clk)
   ram_write <= (cycle_advance_L && fifo_read_in_progress);

wire ram_write_strobe = ~ram_writeD & ram_write;
wire ram_write_done   = ram_writeD & ~ram_write;
reg ram_writeD;
always @(posedge clk_32)
	ram_writeD <= ram_write;

reg [15:0] fifo_data_out;
wire fifo_data_out_strobe = dma_direction_out?io_data_out_strobe:ram_write_done;

always @(posedge clk_32)
   fifo_data_out <= fifo[fifo_rptr];

always @(posedge clk_32 or posedge fifo_reset) begin
   if(fifo_reset == 1'b1)         fifo_rptr <= 4'd0;
   else if (fifo_data_out_strobe) fifo_rptr <= fifo_rptr + 4'd1;
end

// use fifo output directly as ram data
assign ram_dout = fifo_data_out;

// ==========================================================================
// =============================== internal registers =======================
// ==========================================================================
  
// ================================ DMA sector count ========================
// - register is decremented by one after 512 bytes being transferred
// - cpu can write this register directly
// - io controller can write this via the set_address command
   
// keep track of bytes to decrement sector count register
// after 512 bytes (256 words)
reg [7:0] word_cnt;
reg       sector_done;   
reg 	  sector_strobe;
reg 	  sector_strobe_prepare;

always @(posedge clk_32) begin
   if(dma_scnt_write_strobe) begin
      word_cnt <= 8'd0;
      sector_strobe_prepare <= 1'b0;
      sector_strobe <= 1'b0;
      sector_done <= 1'b0;
   end else begin
      if(cycle_io) begin
	 sector_strobe_prepare <= 1'b0;
	 sector_strobe <= 1'b0;

	 // wait a little after the last word
	 if(sector_done) begin
	    sector_done <= 1'b0;
	    sector_strobe_prepare <= 1'b1;
	    // trigger scnt decrement
	    sector_strobe <= 1'b1;
	 end
      end

      // and ram read or write increases the word counter by one
      if(ram_write_strobe || ram_read_strobe) begin
	 word_cnt <= word_cnt + 8'd1;
	 if(word_cnt == 255) begin
	    sector_done <= 1'b1;
	    // give multiplexor some time ahead ...
	    sector_strobe_prepare <= 1'b1;
	 end
      end
   end 
end

// cpu and io controller can write the scnt register and it's decremented 
// after 512 bytes
wire dma_scnt_write_strobe = 
     cpu_scnt_write_strobe || io_addr_strobe || sector_strobe;
// sector counter doesn't count below 0
wire [7:0] dma_scnt_dec = (dma_scnt != 0)?(dma_scnt-8'd1):8'd0;   
// multiplex new sector count data
wire [7:0] dma_scnt_next = sector_strobe_prepare?dma_scnt_dec:
	   cpu_scnt_write_strobe?cpu_din[7:0]:dio_addr_reg[31:24];

// cpu or io controller set the sector count register
always @(posedge clk_32)
	if (dma_scnt_write_strobe) dma_scnt <= dma_scnt_next;
   
// DMA in progress flag:
// - cpu writing the sector count register starts the DMA engine if
//   dma enable bit 6 in mode register is clear
// - io controller setting the address starts the dma engine
// - changing sector count to 0 (cpu, io controller or counter) stops DMA
// - cpu writing toggling dma direction stops dma
reg dma_in_progress;
wire dma_stop = cpu_dma_mode_direction_toggle;

// dma can be started if sector is not zero and if dma is enabled
// by a zero in bit 6 of dma mode register
wire cpu_starts_dma = cpu_scnt_write_strobe && (cpu_din[7:0] != 0) && !dma_mode[6];
wire ioc_starts_dma = io_addr_strobe && (dio_addr_reg[31:24] != 0);

always @(posedge clk_32 or posedge dma_stop) begin
   if(dma_stop) dma_in_progress <= 1'b0;
   else if (dma_scnt_write_strobe) dma_in_progress <= cpu_starts_dma || ioc_starts_dma || (sector_strobe && dma_scnt_next != 0);   
end

// ========================== DMA direction flag ============================
reg dma_direction_out;  // == 1 when transferring from fpga to io controller
wire dma_direction_set = io_addr_strobe || cpu_dma_mode_direction_toggle;

// bit 8 == 0 -> dma read -> dma_direction_out, io ctrl address bit 23 = dir
wire dma_direction_out_next = cpu_mode_write_strobe?cpu_din[8]:dio_addr_reg[23];

// cpu or io controller set the dma direction   
always @(posedge clk_32)
  if (dma_direction_set) dma_direction_out <= dma_direction_out_next;

// ================================= DMA address ============================
// address can be changed through three events:
// - cpu writes single address bytes into three registers
// - io controller writes address via spi
// - dma engine runs and address is incremented

// dma address is stored in three seperate registers as 
// otherwise verilator complains about signals having multiple driving blocks
reg [7:0] dma_addr_h;
reg [7:0] dma_addr_m;
reg [6:0] dma_addr_l;
wire [22:0] dma_addr = { dma_addr_h, dma_addr_m, dma_addr_l };

reg dma_addr_inc;
always @(posedge clk_32)
   dma_addr_inc <= ram_write_done || ram_read_done;

wire dma_addr_write_strobe = dma_addr_inc || io_addr_strobe;

// address to be set by next write strobe
wire [22:0] dma_addr_next = 
	    cpu_address_write_strobe?{ cpu_din[7:0], cpu_din[7:0], cpu_din[7:1] }:
	    io_addr_strobe?{ dio_addr_reg[22:0] }:
	    (dma_addr + 23'd1);

always @(posedge clk_32) begin
	if (dma_addr_write_strobe_l) dma_addr_l <= dma_addr_next[ 6: 0];
	if (dma_addr_write_strobe_m) dma_addr_m <= dma_addr_next[14: 7];
	if (dma_addr_write_strobe_h) dma_addr_h <= dma_addr_next[22:15];
end

// dma address low byte
wire dma_addr_write_strobe_l = dma_addr_write_strobe || (cpu_address_write_strobe && (cpu_addr == 3'h6));
   
// dma address mid byte
wire dma_addr_write_strobe_m = dma_addr_write_strobe || (cpu_address_write_strobe && (cpu_addr == 3'h5));
   
// dma address hi byte
wire dma_addr_write_strobe_h = dma_addr_write_strobe || (cpu_address_write_strobe && (cpu_addr == 3'h4));

// dma address is used directly to address the ram
assign ram_addr = dma_addr;

// ====================================================================
// ======================= Client to IO controller ====================
// ====================================================================

wire [4:0] bcnt = dio_status_index - 1'd1;

// dma status byte as signalled to the io controller
wire [7:0] dma_io_status =
	   (bcnt == 0)?dma_addr[22:15]:
	   (bcnt == 1)?dma_addr[14:7]:
	   (bcnt == 2)?{ dma_addr[6:0], dma_direction_out }:
	   (bcnt == 3)?dma_scnt:
	   // 5 bytes FDC status
	   ((bcnt >= 4)&&(bcnt <= 8))?fdc_status_byte:
	   // 11 bytes ACSI status
	   ((bcnt >= 9)&&(bcnt <= 19))?acsi_status_byte:
		// DMA debug signals
	   (bcnt == 20)?8'ha5:
	   (bcnt == 21)?{ fifo_rptr, fifo_wptr}:
	   (bcnt == 22)?{ 4'd0, fdc_irq, acsi_irq, ioc_br_clk, dma_in_progress }:
	   (bcnt == 23)?dio_dma_status:
	   (bcnt == 24)?dma_mode[8:1]:
	   (bcnt == 25)?fdc_irq_count:
	   (bcnt == 26)?acsi_irq_count:
	   8'h00;

assign dio_status_in = dma_io_status;
assign dio_data_out_reg = fifo_data_out;

wire io_addr_strobe = dio_addr_strobe ^ dio_addr_strobeD;
wire io_data_in_strobe = dio_data_in_strobe ^ dio_data_in_strobeD;
wire io_data_out_strobe = dio_data_out_strobe ^ dio_data_out_strobeD;
wire io_dma_ack = dio_dma_ack ^ dio_dma_ackD;
wire io_dma_nak = dio_dma_nak ^ dio_dma_nakD;

reg dio_addr_strobeD;
reg dio_data_in_strobeD;
reg dio_data_out_strobeD;
reg dio_dma_ackD;
reg dio_dma_nakD;

always@(posedge clk_32) begin
	dio_addr_strobeD <= dio_addr_strobe;
	dio_data_in_strobeD <= dio_data_in_strobe;
	dio_data_out_strobeD <= dio_data_out_strobe;
	dio_dma_ackD <= dio_dma_ack;
	dio_dma_nakD <= dio_dma_nak;
end

reg ioc_br = 1'b0;

endmodule
