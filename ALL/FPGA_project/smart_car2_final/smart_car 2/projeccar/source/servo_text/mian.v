module mian(	
	input					sys_clk,
	input					rst_n,
	input	wire [1:0]	    done,
    input   wire [1:0]      state_input,
//	output				gear1,			
//	output				gear2,
//	output				gear3,
//	output   			gear4
	output	wire [1:0]	    flag,

    output  reg  [15:0]     x,
    output  reg  [15:0]     y,
    output  reg  [15:0]     z
//  output	reg  [7:0]	    angle1,
//	output	reg  [7:0]	    angle2,
//	output	reg  [7:0]	    angle4

    
);

// 计时器逻辑0.8s timer1
always @(posedge sys_clk or negedge rst_n) begin
    if (!rst_n) timer1 <= 0;
    else begin
        if ((state_input == 2'b01)||(state_input == 2'b10)) timer1 <= timer1 + 1;
        else timer1 <= 0;
    end
end

always @(posedge sys_clk or negedge rst_n) begin
    if (!rst_n) begin timer_0s8_done <= 0;  end
    else begin
        if (timer1 >= 28'd40_000_000) begin
		      timer_0s8_done <= 1'b1; 
		  end
		  else begin
		      timer_0s8_done <= 1'b0;
		  end
    end
end

//	   localparam           TIME_3D6S = 180_000_000;	//3.6S
//		localparam			   TIME_1D8S = 90_000_000;		//1.8S
//		localparam			   TIME_20ms = 1_000_000;		//20ms
//		reg  [31:0]         cnt;
//		
//	always @(posedge sys_clk or negedge rst_n)begin
//		    if(rst_n==1'b0)begin
//		        cnt <= 0;
//		    end
//		    else if(1) begin
//		        if(cnt >= TIME_3D6S*8)begin
//		            cnt <= 0;
//						end
//		        else    cnt <= cnt + 1;
//		    end
//		end
		
//		reg  [7:0]			  angle1;
//		reg  [7:0]			  angle2;
//		reg  [7:0]			  angle4;
//		wire [1:0]			  done;
		reg  [1:0]			  flag_temp;
//		wire [1:0]			  flag;
        reg                   full_cycle;
        reg  [28:0]           timer1;
        reg                   timer_0s8_done;
		
//模拟控制		
	always @(posedge sys_clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        flag_temp <= 2'b00;
                full_cycle <= 1'b0;
		    end
			 else if(done==2'b10 && !full_cycle)begin	//放置完成停止动作
					flag_temp<=2'b00;
                    full_cycle<=1'b1;
			 end
		     else begin
			//  if(cnt == 1)begin//模拟，开始输入状态抓取物体，角度
                if((state_input == 2'b01) && !(done==2'b01))begin//测试用
//			        angle1<=10;
//					angle2<=170;
//					angle4<=160;
			      	x<=-40;
					y<=45;
					z<=65;
                    if(timer_0s8_done == 1'b1) flag_temp <= 2'b01;
				  end
			 // else if(cnt==TIME_3D6S*3)begin//等待小车运动到指定位置
                else if((done == 2'b01))begin//测试用
                     if(state_input == 2'b00)begin
				  flag_temp <= 2'b00;
                  full_cycle<=1'b0;end
                    else if(state_input==2'b10)
                        begin  /*angle1<=20; angle2<=90; angle4<=170;*/x<=-60;y<=20;z<=110;if(timer_0s8_done == 1'b1) flag_temp <= 2'b10; end
                    else
                        begin flag_temp <= 2'b00; full_cycle<=1'b0; end
			     end
                else begin flag_temp <= 2'b00; full_cycle<=1'b0; end
			 // else if(cnt==TIME_3D6S*4)begin//到达指定位置，输入指令放置物体，角度
//                else if((done == 2'b01))begin//测试用
//                     if
//					flag_temp<=2'b10;
//					angle1<=20;
//					angle2<=90;
//					angle4<=170;
//				   end			
			end	   
		end				
		
	assign flag=flag_temp;	
		
// servo_ctr servo_ctr_inst(	
//	.sys_clk		(sys_clk),
//	.rst_n		(rst_n),
//	.flag			(flag),
//	.angle1_in  (angle1),
//	.angle2_in  (angle2),
//	.angle4_in  (angle4),
//	.done			(done),
//	.gear1		(gear1),
//	.gear2		(gear2),
//	.gear3		(gear3),
//	.gear4		(gear4)
//);
endmodule 