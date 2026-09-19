module   inf_rcv
(
		input wire clk,
		input wire rst_n,
		input wire inf_in,
		
		output reg [7:0]	data,
		output reg 		repeat_en
);
parameter	IDLE 		=  5'b0_0001,
			TIME_9MS 	=  5'b0_0010,
			ARBIT		=  5'b0_0100,
			DATA		=  5'b0_1000,
			REPEAT		=  5'b1_0000;

parameter	CNT_560US_MIN  = 19'd20_000 ,
			CNT_560US_MAX  = 19'd35_000 ,
			CNT_1_69MS_MIN = 19'd80_000 ,
			CNT_1_69MS_MAX = 19'd90_000 ,
			CNT_2_25MS_MIN = 19'd100_000,
			CNT_2_25MS_MAX = 19'd125_000,
			CNT_4_5MS_MIN  = 19'd175_000,
			CNT_4_5MS_MAX  = 19'd275_000,
			CNT_9MS_MIN    = 19'd400_000,
			CNT_9MS_MAX    = 19'd490_000;

reg [4:0] state ;
reg 	inf_in_dly1;
reg 	inf_in_dly2;
wire 	inf_in_fall;
reg [18:0]	cnt;
wire	inf_in_rise;
reg		flag_9ms;
reg		flag_4_5ms;
reg [5:0]	cnt_data;
reg 	flag_560us;
reg		flag_1_69ms;
reg [31:0]	data_reg;
reg 	flag_2_25ms ;

always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		state	<= IDLE;
	else	case(state)
		IDLE:	if(inf_in_fall == 1'b1)
					state <= TIME_9MS;
				else
					state <= IDLE;
		TIME_9MS:  if((inf_in_rise == 1'b1) && (flag_9ms == 1'b1))
					state <= ARBIT;
					else if((inf_in_rise == 1'b1) && (flag_9ms == 1'b0))
					state <= IDLE;
					else 
					state <= TIME_9MS;
		ARBIT:	if((inf_in_fall == 1'b1) && (flag_2_25ms == 1'b1))
					state <= REPEAT;
				else if((inf_in_fall == 1'b1) && (flag_4_5ms == 1'b1))
					state <= DATA;
				else if((inf_in_fall == 1'b1) && (flag_2_25ms == 1'b0)&&(flag_4_5ms == 1'b0))
					state <= IDLE;
				else
					state <= ARBIT;
		DATA:	if((inf_in_rise == 1'b1) && (flag_560us == 1'b0))
					state <= IDLE;
				else if((inf_in_fall == 1'b1) && (flag_560us == 1'b0)&& (flag_1_69ms == 1'b0))
					state <= IDLE;
				else if((inf_in_rise == 1'b1) && (cnt_data == 6'd32))
					state <= IDLE;
		REPEAT:	if((inf_in_rise == 1'b1))
					state <= IDLE;
				else 
					state <= REPEAT;
		default: state <= IDLE;
	endcase

always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		begin
			inf_in_dly1 <= 1'b0;
			inf_in_dly2 <= 1'b0;
		end
	else 
		begin
			inf_in_dly1 <= inf_in;
			inf_in_dly2 <= inf_in_dly1;
		end

assign inf_in_fall = (inf_in_dly1 == 1'b0) && (inf_in_dly2 == 1'b1);
assign inf_in_rise = (inf_in_dly1 == 1'b1) && (inf_in_dly2 == 1'b0);

always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		cnt <= 19'd0;
	else 
		case(state)
			IDLE:	cnt <= 19'd0;
			TIME_9MS:if((inf_in_rise == 1'b1) && (flag_9ms == 1'b1))
						cnt <= 19'd0;
					 else
						cnt <= cnt + 1'b1;
			ARBIT:	if((inf_in_fall == 1'b1) && ((flag_4_5ms == 1'b1) || (flag_2_25ms == 1'b1)))
						cnt <= 19'd0;
					else
						cnt <= cnt + 1'b1;
			DATA:	if((inf_in_rise == 1'b1) && (flag_560us == 1'b1))
						cnt <= 19'd0;
					else if ((inf_in_fall == 1'b1) && ((flag_1_69ms == 1'b1) || (flag_560us == 1'b1)))
						cnt <= 19'd0;
					else
						cnt <= cnt + 1'b1;
			default:	cnt <= 19'd0;
		endcase
						
always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		flag_9ms <= 1'b0;
	else if((state == TIME_9MS) && (cnt >= CNT_9MS_MIN) && (cnt <= CNT_9MS_MAX))
		flag_9ms <= 1'b1;				
	else 
		flag_9ms <= 1'b0;
						
always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		flag_4_5ms <= 1'b0;
	else if((state == ARBIT) && (cnt >= CNT_4_5MS_MIN) && (cnt <= CNT_4_5MS_MAX))
		flag_4_5ms <= 1'b1;				
	else 
		flag_4_5ms <= 1'b0;		

always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		flag_560us <= 1'b0;
	else if((state == DATA) && (cnt >= CNT_560US_MIN) && (cnt <= CNT_560US_MAX))
		flag_560us <= 1'b1;				
	else 
		flag_560us <= 1'b0;	

always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		flag_1_69ms <= 1'b0;
	else if((state == DATA) && (cnt >= CNT_1_69MS_MIN) && (cnt <= CNT_1_69MS_MAX))
		flag_1_69ms <= 1'b1;				
	else 
		flag_1_69ms <= 1'b0;	

always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		flag_2_25ms <= 1'b0;
	else if((state == ARBIT) && (cnt >= CNT_2_25MS_MIN) && (cnt <= CNT_2_25MS_MAX))
		flag_2_25ms <= 1'b1;				
	else 
		flag_2_25ms <= 1'b0;	
		
always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		cnt_data <= 6'd0;
	else if ((inf_in_rise == 1'b1) && (cnt_data == 6'd32))
		cnt_data <= 6'd0;
	else if((inf_in_fall == 1'b1) && (state == DATA))
		cnt_data <= cnt_data + 1'b1;
	else
		cnt_data <= cnt_data;

always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)
		data_reg <= 32'b0;
	else if((state == DATA)&&(inf_in_fall == 1'b1)&&(flag_560us == 1'b1))
		data_reg[cnt_data] <= 1'b0;
	else if((state == DATA)&&(inf_in_fall == 1'b1)&&(flag_1_69ms == 1'b1))
		data_reg[cnt_data] <= 1'b1;
	else
		data_reg <= data_reg;
		
always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)	
		data <= 8'b0;
	else if((cnt_data == 6'd32) && (~data_reg[23:16] == data_reg[31:24]) && (~data_reg[15:8] == data_reg[7:0]))
		data <= data_reg[23:16];
	else 
		data <= data;
		
always@(posedge clk or negedge rst_n)
	if(rst_n == 1'b0)	
		repeat_en <= 1'b0;
	else if((state == REPEAT) && (~data_reg[23:16] == data_reg[31:24]))
		repeat_en <= 1'b1;
	else repeat_en <= 1'b0;	

endmodule