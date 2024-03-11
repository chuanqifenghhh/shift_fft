module AD9764_Code(
	input clk,
	input Fre_Key,
	input Sel_Key,
	
	input key_1,
	input key_2,
	input rst_n,
   output reg a1,//100hz
   output  a2, //300hz
   output  a3,  //500hz
	
	output dac_clk1,
	output dac_clk2,
	output [13:0] dac_data1,
	output [13:0] dac_data2
);

wire clk_125m;
wire rst;
wire [2:0] sel;
wire [31:0] fre_k;
wire [11:0] addr;
wire [13:0] wave_z;
wire [13:0] wave_s;
wire [13:0] wave_f;

wire [13:0] dac_data;
wire [13:0] dac_data_2;
reg [13:0] data_buf1;
reg [13:0] data_buf2;
reg s;
reg r;
reg [17:0] count1;
reg [17:0] count2;
reg [17:0] count3;



always@(posedge c0_sig or negedge rst_n)
  if(!rst_n)
  count1 <= 0;
  else if(count1 == 18'd249999)
  count1 <= 0;
  else
  count1 <= count1 + 1;

always@(posedge c0_sig or negedge rst_n)
  if(!rst_n)
  a1 <= 0;
  else if(count1 == 18'd249999)
  a1 <= ~a1;
  else
  a1 <=  a1;



always@(posedge c1_sig or negedge rst_n)
  if(!rst_n)
  count2 <= 0;
  else if(count2 == 18'd249999)
  count2 <= 0;
  else
  count2 <= count2 + 1;

always@(posedge c1_sig or negedge rst_n)
  if(!rst_n)
  s <= 0;
  else if(count2 == 18'd249999)
  s <= ~s;
  else
  s <=  s;
  



  
  
always@(posedge c2_sig or negedge rst_n)
  if(!rst_n)
  count3 <= 0;
  else if(count3 == 18'd249999)
  count3 <= 0;
  else
  count3 <= count3 + 1;

always@(posedge c2_sig or negedge rst_n)
  if(!rst_n)
  r <= 0;
  else if(count3 == 18'd249999)
  r <= ~r;
  else
  r <=  r;  

reg [8:0] shift_data_1;
reg [8:0] shift_data_2;
always@(posedge c0_sig or negedge rst_n)
  if(!rst_n)begin
  shift_data_1 <= 9'd210;
  shift_data_2 <= 9'd240;
  end else if(key_1)begin
  shift_data_1 <= 9'd30;
  shift_data_2 <= 9'd235;
  end else if(key_2)begin
  shift_data_1 <= 9'd210;
  shift_data_2 <= 9'd240;
  end else begin
  shift_data_1 <= shift_data_1;
  shift_data_2 <= shift_data_2;
  end
  
  
  

//30 235
PhaseShift PhaseShift_inst1(
.clk(clk) ,					//clk
.rst_n(rst_n),				//rest
.din_fre(300),	//input signal clock  frequency,HZ
.phase_angle(shift_data_1) ,//phase shift angle
.din(s) ,			//input signal
.dout(a2) 		//output signal
);

PhaseShift PhaseShift_inst2(
.clk(clk) ,					//clk
.rst_n(rst_n),				//rest
.din_fre(500),	//input signal clock  frequency,HZ
.phase_angle(shift_data_2) ,//phase shift angle
.din(r) ,			//input signal
.dout(a3) 		//output signal
);


















































// 方波数据
assign wave_f = addr[11] ? 14'b11_1111_1111_1111 : 14'b00_0000_0000_0000;

assign dac_data1 = data_buf1;
assign dac_data2 = data_buf2;

pll	pll_inst(
	.areset ( ~rst_n ),
	.inclk0 ( clk ),
	.c0 ( c0_sig ),
	.c1 ( c1_sig ),
	.c2 ( c2_sig ),
	.locked (  )
	);

// 锁相环
PLL U_PLL(
	.inclk0(clk),
	.c0(clk_125m),
	.c1(dac_clk1),
	.c2(dac_clk2),
	.locked(rst)
);

// 按键检测控制
key_con	u_key_con(
	.clk(clk_125m),
	.rst_n(rst),
	.key1_in(Fre_Key),
	.key2_in(Sel_Key),
	.sel_wave(sel),
	.fre_k(fre_k)
);

// 累加器
add_32bit	u_add_32bit(
	.clk(clk_125m),
	.rst(rst),
	.fr_k(fre_k),
	.adder(addr)
);


// 三角信号表

ROM_Sin	u_ROM_Sin(
	.clock(clk_125m),
	.address(addr),
	.q(wave_z)
);
// 正弦信号表


ip	ip_inst (
	.address ( addr ),
	.clock ( clk_125m ),
	.q ( wave_s )
	);


// 输出波形选择
sel_wave u_sel_wave(
	.clk(clk_125m),
	.rst_n(rst),
	.sel(sel),
	.da_ina(wave_z),
	.da_inb(wave_s),
	.da_inc(wave_f),
	.da_out(dac_data),
	.da_out_2(dac_data_2)
);

// 硬件电路输出反相，即数字量0对应最大模拟量输出，数字亮4095对应最小模拟量输出。
// 通过16383-dac_data即可调整过来。
// 两通道数据保持一致输出。
always @(posedge clk_125m) 
begin 
	data_buf1 <= 14'h3FFF - dac_data;
	data_buf2 <= 14'h3FFF - dac_data_2;
end 

endmodule 


