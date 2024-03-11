// 波形选择
module sel_wave(
	input clk,
	input rst_n,
	input [2:0] sel,
	input [13:0] da_ina,
	input [13:0] da_inb,
	input [13:0] da_inc,
	output [13:0] da_out,
	output [13:0] da_out_2
);

reg [13:0] da_out_reg;
assign da_out = da_out_reg;

always @(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		da_out_reg <= 14'd0;
	end
	else
	begin 
	  da_out_reg <= da_ina;
	end
end


reg [13:0] da_out_reg_2;
assign da_out_2 = da_out_reg_2;

always @(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		da_out_reg_2 <= 14'd0;
	end
	else
	begin 
	  da_out_reg_2 <= da_inb;
	end
end








endmodule 