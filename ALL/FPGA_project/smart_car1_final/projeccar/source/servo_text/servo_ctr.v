module servo_ctr(	
	input					sys_clk,
	input					rst_n,
	input	[1:0]			flag,			//操作标志位01为抓，10为放
	input [7:0]			angle1_in,  //右舵机
	input [7:0]			angle2_in,	//底舵机
	input [7:0]			angle4_in,	//左舵机
	output[1:0]			done,			//结束运动标志位01为抓，10为放

	input	wire		ack1,
	input	wire 		ack2,		
	input	wire 		ack3,
	input	wire 		ack4,

	output	reg  [7:0]    angle1,
	output	reg  [7:0]    angle2,
	output	reg  [7:0]    angle3,
	output	reg  [7:0]    angle4

//	output				gear1,			
//	output				gear2,
//	output				gear3,
//	output   			gear4
);

	   localparam           TIME_3D6S = 180_000_000;	//3.6S
		localparam			   TIME_1D8S = 90_000_000;		//1.8S
		localparam			   TIME_20ms = 1_000_000;		//20ms
//		reg  [7:0]			  angle1;
//		reg  [7:0]			  angle2;
//		reg  [7:0]			  angle3;
//		reg  [7:0]			  angle4;
		
		reg  [7:0]			  angle_Target1;
		reg  [7:0]			  angle_Target2;
		reg  [7:0]			  angle_Target4;
		reg  [7:0]			  angle1_move;
		reg  [7:0]				angle1_move2;
		reg  [7:0]			  angle4_move;	
		reg  [1:0]			  state;
		
		reg  [29:0]         cnt;
		reg  [1:0]			  done_temp;
//		wire					  ack1;
//		wire 					  ack2;		
//		wire 					  ack3;
//		wire 					  ack4;	


	
	always  @(posedge sys_clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        angle1_move <= 8'd0;
		    end
			else if(angle1_in > 80)begin
				angle1_move<=8'd120;
			 end			 
			 else if(angle1_in<10)begin
			  angle1_move2<=8'd0;
			  end
			 else begin 
				angle1_move<=angle1_in+8'd40;
				angle1_move2<=angle1_in-8'd10;
			 end
		end
		
		
	always  @(posedge sys_clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
				  angle4_move <= 8'd0;
		    end
			else if(angle4_in<50)begin
				angle4_move<=8'd50;
			 end
//			 else if(angle4_in>140)begin
//					
//				end
			 else begin 
				angle4_move<=angle4_in-8'd50;
			 end
		end	
			
	always  @(posedge sys_clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        cnt <= 0;
		    end
		    else if(flag) begin
		        if(cnt >=TIME_3D6S*3+TIME_1D8S)begin
		            cnt <= 0;
						end
		        else    cnt <= cnt + 1;
		    end
			 else cnt<= 0;
		end 
			
	always @(posedge sys_clk or negedge rst_n)begin
		if(rst_n==1'b0)begin
			angle_Target1<=90;
			angle_Target2<=90;
			state<=2'b10;
			angle_Target4<=90;
			done_temp<=2'b00;
			end	
		//初始	
		else if(flag==2'b01)begin	
		 if(cnt==TIME_20ms*2)begin
			 angle_Target1<=90;
			 angle_Target2<=90;
			 state<=2'b10;
			 angle_Target4<=90;		
		    end		 
		else if(cnt == TIME_1D8S)begin
			 angle_Target1<=30;
			 angle_Target4 <= angle4_move;																		 
			 end
		else if(cnt == TIME_1D8S+30*TIME_20ms)begin
			 angle_Target2 <= angle2_in;																		 
			 end	 
		else if(cnt == TIME_1D8S+75*TIME_20ms)begin
			 angle_Target1 <= angle1_move; 																										 
			 end 
		else if(cnt == (TIME_3D6S))begin
			 state <= 2'b01;										
			 end	 	 
		else if(cnt == (TIME_3D6S+20*TIME_20ms))begin										
			 angle_Target1 <= angle1_in;
			 angle_Target4 <= angle4_move+8'd10;			 
			 end
		else if(cnt == (TIME_3D6S+40*TIME_20ms))begin										
			 angle_Target4 <= angle4_in;										
			 end	 
		else if(cnt == (TIME_3D6S+70*TIME_20ms))begin
			 state<=2'b10;															
			 end 
		else if(cnt == (TIME_3D6S+TIME_1D8S))begin
			 angle_Target1 <= angle1_move;														
			 end	 
		else if(cnt ==(TIME_3D6S+TIME_1D8S+20*TIME_20ms))begin										
			 angle_Target4 <= angle4_move;
			 angle_Target1 <= 8'd10;		 
			 end
		else if(cnt == (TIME_3D6S*2+10*TIME_20ms))begin 										
			 angle_Target4 <= 8'd90;
			 angle_Target2<=90;
			 												
			 done_temp<=2'b01;
		end		
	end
	
	else if(flag==2'b10)begin
		 if(cnt==TIME_20ms*2)begin
			 angle_Target2<=angle2_in;    								 
			 end
		else if(cnt==TIME_1D8S)begin
			 angle_Target1 <= angle1_move;
			 angle_Target4 <= angle4_move;
			 end	 
		else if(cnt==TIME_1D8S+55*TIME_20ms)begin
			 angle_Target4 <= angle4_in;
			 angle_Target1 <= angle1_in;
			 end
		else if(cnt==(TIME_3D6S))begin
			 state<=2'b01;
			 end
		else if(cnt==(TIME_3D6S+20*TIME_20ms))begin
			 angle_Target1<=angle1_move2;
			 angle_Target4<=angle4_move+8'd20;
			 end	 
		else if(cnt==(TIME_3D6S+30*TIME_20ms))begin
			 angle_Target4<=angle4_move;
			 end	 
		else if(cnt==(TIME_3D6S+60*TIME_20ms))begin
			 angle_Target2<=90;
			 state<=2'b10;
			 angle_Target4<=8'd40;
			 end
		else if(cnt==(TIME_3D6S+TIME_1D8S+60*TIME_20ms))begin
			 angle_Target1<=8'd10;
			 angle_Target4<=8'd90;
			 done_temp<=2'b10;
			 end	 	 
	end
end			
			
		
assign done=done_temp;
		
		always  @(posedge sys_clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
				  angle1 <= 8'd60;

		    end
		        else if(ack1)begin										  
						if(angle1==angle_Target1)begin					//当目标角度大于当前角度时，angle递减
							angle1<=angle1+8'd0;
	
							end
						else if(angle1<angle_Target1)begin			//当目标角度小于当前角度时，angle递增
							angle1<=angle1 + 8'd2;
						
							end
						else begin	   								//当目标角度与当前角度相等时不变。
							angle1<=angle1-8'd2;
						
							end
		    end
		end	
			
		always  @(posedge sys_clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
				  angle2 <= 8'd90;
			
		    end
		        else if(ack2)begin										  
						if(angle2==angle_Target2)begin				//当目标角度大于当前角度时，angle递减
							angle2<=angle2+8'd0;
					
							end
						else if(angle2<angle_Target2)begin			//当目标角度小于当前角度时，angle递增
							angle2<=angle2 + 8'd2;
				
							end
						else begin 		   								//当目标角度与当前角度相等时不变。
							angle2<=angle2-8'd2;
						
							end
		    end
		end			
		
		always  @(posedge sys_clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
				  angle4 <= 8'd70;
			
		    end
		        else if(ack4)begin										  
						if(angle4==angle_Target4)begin					//当目标角度大于当前角度时，angle递减
							angle4<=angle4+8'd0;
						
							end
						else if(angle4<angle_Target4)	begin		//当目标角度小于当前角度时，angle递增
							angle4<=angle4 + 8'd2;

							end
						else begin 		   								//当目标角度与当前角度相等时不变。
							angle4<=angle4-8'd2;
					
							end
		    end
		end		
		
	  always @(posedge sys_clk or negedge rst_n)begin
			if(rst_n==1'b0)begin
				angle3<=8'd130;
			end		
			else if(state==2'b01)begin
				angle3<=8'd10;//开
				end
			else if(state==2'b10)begin
				angle3<=8'd135;//关
			end
	  end 
 
	
//servo	servo1(
//	.sys_clk			(sys_clk),
//	.rst_n			(rst_n),	
//	.gear_req		(1'b1),				//舵机转动请求
//	.angle			(angle1),					//转动角度
//	.gear_ack		(ack1),				//舵机转动完成
//	.gear				(gear1)
//);
//
//servo	servo2(
//	.sys_clk			(sys_clk),
//	.rst_n			(rst_n),	
//	.gear_req		(1'b1),				//舵机转动请求
//	.angle			(angle2),					//转动角度
//	.gear_ack		(ack2),				//舵机转动完成
//	.gear				(gear2)
//);
//
//servo	servo3(
//	.sys_clk			(sys_clk),
//	.rst_n			(rst_n),	
//	.gear_req		(1'b1),				//舵机转动请求
//	.angle			(angle3),					//转动角度
//	.gear_ack		(ack3),				//舵机转动完成
//	.gear				(gear3)
//);
//
//servo	servo4(
//	.sys_clk			(sys_clk),
//	.rst_n			(rst_n),	
//	.gear_req		(1'b1),				//舵机转动请求
//	.angle			(angle4),					//转动角度
//	.gear_ack		(ack4),				//舵机转动完成
//	.gear				(gear4)
//);
//
endmodule 
