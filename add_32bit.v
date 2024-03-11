module add_32bit (
	input clk,
	input rst,
	input [31:0] fr_k,
	output [11:0] adder
);

reg  [31:0] add;

always @(posedge clk) 
begin
	if (!rst) 
	begin
		add <= 32'd0;
	end
	else 
	begin
		add <= add + fr_k;
	end
end

assign   adder = add[31:20];

endmodule 