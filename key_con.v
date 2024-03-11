module key_con (
	input clk, 
	input rst_n,
	input key1_in,
	input key2_in,
	output [2:0] sel_wave,
	output [31:0] fre_k
);

reg  [31:0]   fre;
reg  [1:0]cnt;
reg  [2:0]    sel;



initial
begin
	fre <= 32'd138;			// 100Hz
	cnt <= 2'd0;
end

wire             key1_out;
wire             key2_out;

// delay 
key_delay u_key1_delay(
	.clk(clk),
	.kin(key1_in),
	.kout(key1_out)
);

// delay 
key_delay u_key2_delay(
	.clk(clk),
	.kin(key2_in),
	.kout(key2_out)
);

// 频率调整
always @(negedge key1_out) 
begin
	if(fre<32'd3436)				// 100hz
		fre <= fre + 32'd3436;
	else
		fre <= 32'd3436;
end

// 输出波形选择
always @(negedge key2_out) 
begin 
	if(cnt < 3) 
	begin
		cnt <= cnt + 1'b1;
	end
	else 
	begin 
		cnt <= 2'd0;
	end 
end

always @(posedge clk) begin 
	case (cnt) 
		2'b00 : sel <= 3'b110;
		2'b01 : sel <= 3'b101;
		2'b10 : sel <= 3'b011;
		default : sel <= 3'b110;
	endcase 
end 

assign  fre_k = fre;
assign  sel_wave = sel;

endmodule 