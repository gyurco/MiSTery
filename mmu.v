module mmu (
	// cpu register interface
	input 		 clk,
	input        clk_en,
	input 		 reset,
	input [7:0] 	 din,
	input 		 sel,
	input 		 ds,
	input 		 rw,
	output reg [7:0] dout
);

reg [7:0] memconfig;

always @(sel, ds, rw, memconfig) begin
	dout = 8'd0;
	if(sel && ~ds && rw)
		dout = memconfig;
end

always @(posedge clk) begin
	if(reset)
		memconfig <= 8'h00;
	else begin
		if(clk_en && sel && ~ds && ~rw)
			memconfig <= din;
	end
end

endmodule