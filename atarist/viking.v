//
// viking.v
// 
// Atari ST(E) Viking/SM194
// https://github.com/mist-devel/mist-board
// 
// Copyright (c) 2013-2015 Till Harbaum <till@harbaum.org> 
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

// The viking card does not have its own CPU interface as it is not 
// configurable in any way. It just etches data from ram and displays
// it on screen.

module viking (
	input pclk,                   // 128 MHz pixel clock

	// memory interface
	input             himem,      // use memory behind rom
	input             clk_8_en,   // 8 MHz bus clock
	input [1:0]       bus_cycle,  // bus-cycle for bus access sync
	output reg [22:0] addr,       // video word address
	output            read,       // video read cycle
	input [63:0]      data,       // video data read

	// VGA output (multiplexed with sm124 output in top level)
	output         hs,
	output         vs,
	output         hb,
	output         vb,
	output [3:0]   r,
	output [3:0]   g,
	output [3:0]   b
);

localparam BASE    = 23'h600000;   // c00000
localparam BASE_HI = 23'h740000;   // e80000
		
// total width must be multiple of 64, so video runs synchronous
// to main bus
		
// Horizontal timing
// HBP1 |                    H              | HFP | HS | HBP2
// -----|XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX|-----|____|-----
// HBP1 is used for prefetch
	
// 1280x
localparam H    = 1280;
localparam HFP  = 88;
localparam HS   = 136;
localparam HBP1 = 64;
localparam HBP2 = 160;

// Vertical timing
//                     V              | VFP | VS | VBP
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX|-----|____|-----

// x1024
localparam V    = 1024;
localparam VFP  = 9;
localparam VS   = 4;
localparam VBP  = 9;

assign read = (bus_cycle == 1) && me;  // memory enable can directly be used as a ram read signal

// ---------------------------------------------------------------------------
// --------------------------- internal state counter ------------------------
// ---------------------------------------------------------------------------

reg [3:0] t;
always @(posedge pclk) begin
	reg clk_8_enD;

	clk_8_enD <= clk_8_en;
	// clk_8_en is at the middle of the cycle
	if (~clk_8_enD & clk_8_en) t <= 4'hD; else t <= t + 1'd1;
end

// create internal bus_cycle signal
reg [5:0] bus_cycle_L;
always @(posedge pclk)
	bus_cycle_L <= { bus_cycle, t };


// --------------- horizontal timing -------------
reg[10:0] h_cnt;   // 0..2047
assign hs = ((h_cnt >= HBP1+H+HFP) && (h_cnt < HBP1+H+HFP+HS))?1'b0:1'b1;
always@(posedge pclk) begin
	if(h_cnt==HBP1+H+HFP+HS+HBP2-1) begin
		// make sure a line starts with the "video" bus cyle (0)
		// cpu has cycles 1 and 2
		if(bus_cycle_L == 6'h2e)
			h_cnt<=0;
	end else
		h_cnt <= h_cnt + 1'd1;
end

// --------------- vertical timing -------------
reg[10:0] v_cnt;   // 0..2047
assign vs = ((v_cnt >= V+VFP) && (v_cnt < V+VFP+VS))?1'b0:1'b1;
always@(posedge pclk) begin
	if(h_cnt==HBP1+H+HFP+HS+HBP2-1) begin
		if(v_cnt==V+VFP+VS+VBP-1)  v_cnt <= 0; 
		else 								v_cnt <= v_cnt+1'd1;
	end
end

reg [63:0] input_latch;
reg [63:0] shift_register;

// ---------------- memory timing ----------------
always@(posedge pclk) begin
	// last line on screen
	if(v_cnt == V+VFP+VS+VBP-2)
		addr <= himem?BASE_HI:BASE;
	else if(me && bus_cycle_L == 6'h1e)   // directly after read 
		addr <= addr + 23'd4;              // advance 4 words (64 bits)
		
	if(me && (bus_cycle_L == 6'h1e))
		input_latch <= data;
		
	if(bus_cycle_L == 6'h2e)
		// reorder words 1:2:3:4 -> 4:3:2:1
		shift_register <= 
			{  input_latch[15:0], input_latch[31:16], 
			  input_latch[47:32], input_latch[63:48] };
	else	
		shift_register[63:1] <= shift_register[62:0];
end
 
// memory enable (data is being read from memory)
wire me	= (v_cnt < V)&&(h_cnt < H);
// display enable (data is being displayed)
wire hde = (h_cnt >= HBP1)&&(h_cnt < HBP1+H);
wire vde = v_cnt < V;
wire de  = hde | vde;
assign hb = ~hde;
assign vb = ~vde;

wire pix = de?(!shift_register[63]):1'b0;

// drive all 12 rgb bits from the data bit
wire [3:0] pix4 = { pix, pix, pix, pix };
assign r = pix4;
assign g = pix4;
assign b = pix4;

endmodule
