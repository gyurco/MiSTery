//
// osd.v
// 
// On Screen Display implementation for the MiST board
// https://github.com/mist-devel
// 
// Copyright (c) 2015 Till Harbaum <till@harbaum.org> 
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

module osd (
	// OSDs pixel clock, should be synchronous to cores pixel clock to
	// avoid jitter.
	input        clk,
	input        ce_pix,

	// SPI interface
	input        sck,
	input        ss,
	input        sdi,

	// VGA signals coming from core
	input  [5:0] R_in,
	input  [5:0] G_in,
	input  [5:0] B_in,
	input        HSync,
	input        VSync,
	
	// VGA signals going to video connector
	output [5:0] R_out,
	output [5:0] G_out,
	output [5:0] B_out
);

parameter OSD_X_OFFSET = 10'd0;
parameter OSD_Y_OFFSET = 10'd0;
parameter OSD_COLOR    = 3'b010;

localparam OSD_WIDTH   = 10'd256;
localparam OSD_HEIGHT  = 10'd128;

// *********************************************************************************
// spi client
// *********************************************************************************

// this core supports only the display related OSD commands
// of the minimig
reg        osd_enable;
(* ramstyle = "no_rw_check" *) reg  [7:0] osd_buffer[2047:0];  // the OSD buffer itself

// the OSD has its own SPI interface to the io controller
always@(posedge sck, posedge ss) begin
	reg  [4:0] cnt;
	reg [10:0] bcnt;
	reg  [7:0] sbuf;
	reg  [7:0] cmd;

	if(ss) begin
		cnt  <= 0;
		bcnt <= 0;
	end else begin
		sbuf <= {sbuf[6:0], sdi};

		// 0:7 is command, rest payload
		if(cnt < 15) cnt <= cnt + 1'd1;
			else cnt <= 8;

		if(cnt == 7) begin
			cmd <= {sbuf[6:0], sdi};

			// lower three command bits are line address
			bcnt <= {sbuf[1:0], sdi, 8'h00};

			// command 0x40: OSDCMDENABLE, OSDCMDDISABLE
			if(sbuf[6:3] == 4'b0100) osd_enable <= sdi;
		end

		// command 0x20: OSDCMDWRITE
		if((cmd[7:3] == 5'b00100) && (cnt == 15)) begin
			osd_buffer[bcnt] <= {sbuf[6:0], sdi};
			bcnt <= bcnt + 1'd1;
		end
	end
end

// *********************************************************************************
// video timing and sync polarity anaylsis
// *********************************************************************************

// horizontal counter
reg  [9:0] h_cnt;
reg  [9:0] hs_low, hs_high;
wire       hs_pol = hs_high < hs_low;
wire [9:0] dsp_width = hs_pol ? hs_low : hs_high;

// vertical counter
reg  [9:0] v_cnt;
reg  [9:0] vs_low, vs_high;
wire       vs_pol = vs_high < vs_low;
wire [9:0] dsp_height = vs_pol ? vs_low : vs_high;

wire doublescan = (dsp_height>350); 

always @(posedge clk) begin
	reg hsD;
	reg vsD;

	if(ce_pix) begin
		hsD <= HSync;

		// falling edge of HSync
		if(!HSync && hsD) begin	
			h_cnt <= 0;
			hs_high <= h_cnt;
		end

		// rising edge of HSync
		else if(HSync && !hsD) begin	
			h_cnt <= 0;
			hs_low <= h_cnt;
			v_cnt <= v_cnt + 1'd1;
		end else begin
			h_cnt <= h_cnt + 1'd1;
		end

		vsD <= VSync;

		// falling edge of VSync
		if(!VSync && vsD) begin	
			v_cnt <= 0;
			vs_high <= v_cnt;
		end

		// rising edge of VSync
		else if(VSync && !vsD) begin	
			v_cnt <= 0;
			vs_low <= v_cnt;
		end
	end
end

// area in which OSD is being displayed
wire [9:0] h_osd_start = ((dsp_width - OSD_WIDTH)>> 1) + OSD_X_OFFSET;
wire [9:0] h_osd_end   = h_osd_start + OSD_WIDTH;
wire [9:0] v_osd_start = ((dsp_height- (OSD_HEIGHT<<doublescan))>> 1) + OSD_Y_OFFSET;
wire [9:0] v_osd_end   = v_osd_start + (OSD_HEIGHT<<doublescan);
reg  [9:0] osd_hcnt;
reg  [9:0] osd_vcnt;

wire osd_de = osd_enable && 
              (HSync != hs_pol) && (h_cnt >= h_osd_start) && (h_cnt < h_osd_end) &&
              (VSync != vs_pol) && (v_cnt >= v_osd_start) && (v_cnt < v_osd_end);

reg  [7:0] osd_byte;
always @(posedge clk) if(ce_pix) begin
	osd_hcnt <= h_cnt - h_osd_start + 2'd2;  // 1+1 pixel offset for osd_byte register
	osd_vcnt <= v_cnt - v_osd_start;
	osd_byte <= osd_buffer[{doublescan ? osd_vcnt[7:5] : osd_vcnt[6:4], osd_hcnt[7:0]}];
end

wire osd_pixel = osd_byte[doublescan ? osd_vcnt[4:2] : osd_vcnt[3:1]];

assign R_out = !osd_de ? R_in : {osd_pixel, osd_pixel, OSD_COLOR[2], R_in[5:3]};
assign G_out = !osd_de ? G_in : {osd_pixel, osd_pixel, OSD_COLOR[1], G_in[5:3]};
assign B_out = !osd_de ? B_in : {osd_pixel, osd_pixel, OSD_COLOR[0], B_in[5:3]};

endmodule
