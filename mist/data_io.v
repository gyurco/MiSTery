//
// data_io.v
//
// Data interface for the Atari ST core on the MiST board. 
// Providing ROM and floppy data up- and download via the MISTs
// own arm7 cpu.
//
// http://code.google.com/p/mist-board/
//
// Copyright (c) 2014-2015 Till Harbaum <till@harbaum.org>
// Copyright (c) 2019 György Szombathelyi
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

module data_io #(parameter ADDR_WIDTH=24, START_ADDR = 0) (
	input 			    clk,
	// io controller spi interface
	input 			    sck,
	input 			    ss,
	input 			    sdi,
	output reg          sdo,

	// MiST settings
	output reg [31:0] ctrl_out,
	// horizontal and vertical screen adjustments
	output reg [15:0] video_adj,

	output reg        addr_strobe,
	output reg [31:0] addr_reg, //31-24 sector count, 23 direction, 22-0 mem address

	// data_in_reg valid
	output reg        data_in_strobe,
	output reg [15:0] data_in_reg,

	// valid before the next data_out_reg required
	// give enough time to advance the pointer and load
	output reg        data_out_strobe,
	input      [15:0] data_out_reg,

	output reg        dma_ack,
	output reg  [7:0] dma_status,

	output reg        dma_nak,

	input       [7:0] status_in,
	output      [4:0] status_index
);

// *********************************************************************************
// spi client
// *********************************************************************************

reg [6:0]      sbuf;
reg [7:0]      data;
reg [2:0]      bit_cnt;
reg [4:0]      byte_cnt;
reg [7:0]      cmd;
reg            odd;

assign status_index = byte_cnt;

localparam MIST_SET_ADDRESS  = 8'h01;
localparam MIST_WRITE_MEMORY = 8'h02;
localparam MIST_READ_MEMORY  = 8'h03;
localparam MIST_SET_CONTROL  = 8'h04;
localparam MIST_GET_DMASTATE = 8'h05;  // reads state of dma and floppy controller
localparam MIST_ACK_DMA      = 8'h06;  // acknowledge a dma command
localparam MIST_BUS_REQ      = 8'h07;  // request bus - not used
localparam MIST_BUS_REL      = 8'h08;  // release bus - not used
localparam MIST_SET_VADJ     = 8'h09;
localparam MIST_NAK_DMA      = 8'h0a;  // reject a dma command

// SPI bit and byte counters
reg [15:0] data_out_reg_r;
reg [ 7:0] status_in_r;

always@(posedge sck or posedge ss) begin
	if(ss == 1) begin
		bit_cnt <= 0;
		byte_cnt <= 0;
		cmd <= 0;
		odd <= 0;
	end else begin
		if (!bit_cnt) begin
			if (odd) data_out_reg_r <= data_out_reg;
			status_in_r <= status_in;
		end
		if(&bit_cnt) begin
			odd <= ~odd;
			if(~&byte_cnt) byte_cnt <= byte_cnt + 1'd1;
			if (!byte_cnt) cmd <= {sbuf, sdi};
		end
		bit_cnt <= bit_cnt + 1'd1;
	end
end

// SPI transmitter FPGA -> IO
always@(negedge sck or posedge ss) begin

	if(ss == 1) begin
		sdo <= 1;
	end else begin
		if(cmd == MIST_READ_MEMORY)
			sdo <= !bit_cnt ? data_out_reg[{ odd, ~bit_cnt }]: data_out_reg_r[{ odd, ~bit_cnt }];
		else
			sdo <= !bit_cnt ? status_in[~bit_cnt] : status_in_r[~bit_cnt];
	end
end

// SPI receiver IO -> FPGA
reg       spi_receiver_strobe_r = 0;
reg       spi_transfer_end_r = 1;
reg [7:0] spi_byte_in;

// Read at spi_sck clock domain, assemble bytes for transferring to clk_sys
always@(posedge sck or posedge ss) begin

	if(ss == 1) begin
		spi_transfer_end_r <= 1;
	end else begin
		spi_transfer_end_r <= 0;

		if(&bit_cnt) begin
			// finished reading a byte, prepare to transfer to clk_sys
			spi_byte_in <= { sbuf, sdi};
			spi_receiver_strobe_r <= ~spi_receiver_strobe_r;
		end else
			sbuf[6:0] <= { sbuf[5:0], sdi };
	end
end

// Process bytes from SPI at the clk_sys domain
always @(posedge clk) begin

	reg        spi_receiver_strobe;
	reg        spi_transfer_end;
	reg        spi_receiver_strobeD;
	reg        spi_transfer_endD;
	reg  [7:0] acmd;
	reg  [9:0] abyte_cnt;
	reg [31:8] latch;
	reg        lo;

	//synchronize between SPI and sys clock domains
	spi_receiver_strobeD <= spi_receiver_strobe_r;
	spi_receiver_strobe  <= spi_receiver_strobeD;
	spi_transfer_endD    <= spi_transfer_end_r;
	spi_transfer_end     <= spi_transfer_endD;

	// strobe is set whenever a valid byte has been received
	if (~spi_transfer_endD & spi_transfer_end) begin
		abyte_cnt <= 0;
		lo <= 0;
	end else if (spi_receiver_strobeD ^ spi_receiver_strobe) begin

		if(~&abyte_cnt)
			abyte_cnt <= abyte_cnt + 1'd1;

		if(!abyte_cnt) begin
			acmd <= spi_byte_in;
			if (spi_byte_in == MIST_NAK_DMA) dma_nak <= ~dma_nak;
		end else begin
			case(acmd)

			MIST_SET_VADJ:
			if (abyte_cnt == 1)      latch[15:8] <= spi_byte_in;
			else if (abyte_cnt == 2) video_adj <= { latch[15:8], spi_byte_in };

			MIST_SET_CONTROL:
			if (abyte_cnt == 1)      latch[31:24] <= spi_byte_in;
			else if (abyte_cnt == 2) latch[23:16] <= spi_byte_in;
			else if (abyte_cnt == 3) latch[15: 8] <= spi_byte_in;
			else if (abyte_cnt == 4) ctrl_out <= { latch[31:8], spi_byte_in };

			MIST_SET_ADDRESS:
			if (abyte_cnt == 1)      latch[31:24] <= spi_byte_in;
			else if (abyte_cnt == 2) latch[23:16] <= spi_byte_in;
			else if (abyte_cnt == 3) latch[15: 8] <= spi_byte_in;
			else if (abyte_cnt == 4) begin
				addr_reg <= { latch[31:8], spi_byte_in };
				addr_strobe <= ~addr_strobe;
			end

			MIST_WRITE_MEMORY:
			begin
				lo <= ~lo;
				if (~lo) latch[15: 8] <= spi_byte_in;
				else begin
					data_in_reg <= { latch[15:8], spi_byte_in };
					data_in_strobe <= ~data_in_strobe;
				end
			end

			MIST_READ_MEMORY:
			begin
				lo <= ~lo;
				if (~lo) data_out_strobe <= ~data_out_strobe;
			end

			MIST_ACK_DMA:
			begin
				dma_ack <= ~dma_ack;
				dma_status <= spi_byte_in;
			end

			endcase;
		end
	end
end

endmodule
