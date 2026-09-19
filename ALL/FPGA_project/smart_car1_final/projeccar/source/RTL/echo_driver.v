/**************
这个模块是用于处理超声波模块在回响过程的高电平，
其高低平持续的时间就是其距离的两倍，我们通过下面公式进行判断
测试距离=(高电平时间*声速(340M/S))/2
高电平的持续时间通过上升沿、下降沿来进行获取
**************/
module echo_driver(
    input           sys_clk         ,
    input           sys_us          ,
    input           sys_rst_n       ,
    input           echo            ,

    output [18:0]   data_o          //检测距离，保留三位小数，*1000实现
);
//检测边沿获取高电平持续时间

	parameter T_MAX = 16'd5_9999;//510cm 对应计数值

	reg				r1_echo,r2_echo; //边沿检测	
	wire			echo_pos,echo_neg; //
	
	reg		[15:00]	cnt		; 
	
	reg		[18:00]	data_r	;
	
	//如果使用clk_us 检测边沿，延时2us，差值过大
	always @(posedge sys_clk or negedge sys_rst_n)begin  
		if(!sys_rst_n)begin  
			r1_echo <= 1'b0;
			r2_echo <= 1'b0;
		end  
		else begin  
			r1_echo <= echo;
			r2_echo <= r1_echo;
		end  
	end
	
	assign echo_pos = r1_echo & ~r2_echo;
	assign echo_neg = ~r1_echo & r2_echo;
	//依据电平计时
	always @(posedge sys_us or negedge sys_rst_n) begin
		if(!sys_rst_n) begin
			cnt <= 16'd0;
		end
		else if(echo) begin//这里使用了高电平开始就不使用上升沿了
			if(cnt == T_MAX) begin//超过测量范围则被判断为不合理的检测
				cnt <= 16'd0;
			end
			else begin
				cnt <= cnt + 16'd1;
			end
		end
		else begin
			cnt <= 16'd0;
		end
	end
	//左移将小数寄存
	always @(posedge sys_clk or negedge sys_rst_n)begin  
		if(!sys_rst_n)begin  
			data_r <= 19'd0;
		end  
		else if(echo_neg)begin  
			data_r <= (cnt << 4) + cnt;
		end  
		else begin  
			data_r <= data_r;
		end  
	end
	
	assign data_o = data_r >> 1;
endmodule