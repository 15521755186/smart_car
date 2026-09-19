module	servo(
	input				sys_clk,
	input				rst_n,
	input				gear_req,				//舵机转动请求
	input[7:0]		angle,					//转动角度
	output			gear_ack,				//舵机转动完成
	output			gear						//舵机IO口
);

localparam	T_20ms	=		'd1000_000;//	周期20ms

reg[20:0]	T_cnt;				//	周期计时器
reg			gear_reg;
assign		gear		=	gear_reg;
assign		gear_ack	= ( T_cnt == T_20ms ) ? 1'b1 : 1'b0;

//周期计数
always@(posedge	sys_clk or negedge rst_n )
begin
	if( rst_n == 1'b0 )
		T_cnt <= 'd0;
	else if( gear_ack == 1'b1 )
		T_cnt <= 'd0;
	else if( gear_req == 1'b1 )
		T_cnt <= T_cnt + 1'b1;
	else
		T_cnt <= 'd0;	
end

//脉冲输出
always@(posedge sys_clk or negedge rst_n)
begin
	if( rst_n == 1'b0 )
		gear_reg <= 1'b0;
	else if( gear_req == 1'b1 && T_cnt < ( angle * 'd555 + 'd25000) )
		gear_reg <= 1'b1;
	else
		gear_reg <= 1'b0;
end

endmodule 