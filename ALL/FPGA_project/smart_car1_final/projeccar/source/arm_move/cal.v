module calculate(
	input 		clk,
	input 		rst_n,
	//输入
	input signed[15:0]   x,//上下
	input signed[15:0]	y,//前后
	input signed[15:0]	z,//左右
	//输出角度
	output[7:0]	angle_x,
	output[7:0]	angle_y,
	output[7:0]	angle_z
);

parameter			  arm_lenght  = 40;//臂向补偿，单位mm
parameter			  base_Height = 50; //底座高度，单位mm

reg signed[15:0]			  x_temp;
reg signed[15:0]			  y_temp;
reg signed[15:0]			  z_temp;

always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        x_temp <= 16'd0;
				  y_temp	<= 16'd0;
				  z_temp <= 16'd0;
		    end
		    else  begin
		     	  x_temp <= x;
				  y_temp	<= y;
				  z_temp <= z;
		    end
		end		
/*************************************************************底角计算*********************************************************/
reg  [31:0]			   base_angle1;
reg  signed [31:0]   x_input1_temp;  // 有符号 X 坐标（32 位）
reg  signed [31:0] 	y_input1_temp;  // 有符号 Y 坐标（32 位）

always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
				  x_input1_temp <= 32'sd0;
				  y_input1_temp <= 32'sd0;
		    end
			 
		    else  begin
				  x_input1_temp <= {{16{x_temp[15]}}, x_temp}<<16;
				  y_input1_temp <= {{16{y_temp[15]}}, y_temp}<<16;
		    end
		end

/***********************************************************大臂角2************************************************************/
reg   [31:0]			base_angle2;
reg [2:0]				angle_flag;
reg signed  [31:0]	x_input2_temp; 		// 有符号 X 坐标（32 位）
reg signed  [31:0] 	y_input2_temp; 		// 有符号 Y 坐标（32 位）

always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        x_input2_temp <= 32'sd0;
				  y_input2_temp <= 32'sd0;
				  angle_flag<=2'b00;
		    end
			else if (base_Height > z_temp) begin
				x_input2_temp <= {{16{short_side[15]}}, short_side[15:0] }<<16; // 保持Q16格式
				y_input2_temp <= {{24{resualt1[7]}}, resualt1[7:0] }<<16;
				angle_flag<=2'b01;
			end 
			else if(base_Height<z_temp)begin
				x_input2_temp <= {{24{resualt1[7]}}, resualt1[7:0] }<<16;
				y_input2_temp <= {{16{short_side[15]}}, short_side[15:0]}<<16;
				angle_flag<=2'b10;
			end
			 else begin
			 	  x_input2_temp <= 32'sd0;
				  y_input2_temp <= 32'sd0;
				  angle_flag<=2'b11;  
			 end
		end	
		
/**********************************************************************************************************************/
reg   					arctan_sel;
reg  signed[31:0]		arctan_inputx;
reg  signed[31:0]		arctan_inputy;
wire [31:0]				arctan_resualt;
wire						vaild;

always @(posedge clk or negedge rst_n) begin
	if(rst_n==1'b0)begin
			arctan_inputx<=32'sd0;
			arctan_inputy<=32'sd0;
			base_angle1<=32'b0;
			base_angle2<=32'b0;
			arctan_sel<=1'b0;
		end
	else if(1)begin
    case (arctan_sel)
		  1'b0:begin
					if(vaild)begin
				   base_angle2<=arctan_resualt;
					arctan_sel<=1'b1;
					end
					else begin
					arctan_inputx<=x_input1_temp;
					arctan_inputy<=y_input1_temp;
					end
				end
		  1'b1:begin
					if(vaild)begin
					base_angle1<=arctan_resualt;
					arctan_sel<=1'b0;
					end
					else begin
					arctan_inputx<=x_input2_temp;
					arctan_inputy<=y_input2_temp;						
					end
				end
        default:	arctan_sel<=1'b0;		
     endcase
	end
end

CordicAtan arctan( 
	 .clk(clk), 				 	//时钟
	 .rst_n(rst_n), 		 		//异步复位
    .x1(arctan_inputx),  		
    .y1(arctan_inputy),
    .valid1(vaild),     		// 有效信号valid
    .atan1(arctan_resualt) 	// 结果
);
//assign angle_z = base_angle1>>16;
/*************************************************************垂直投影*********************************************************/
reg  [15:0]			  vertical_input1_temp;
reg  [7:0]			  resualt_temp1;
reg  [7:0]			  resualt1;				//垂直投影

always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        vertical_input1_temp <= 16'b0;
		    end
		    else begin
		        vertical_input1_temp <= (x_temp*x_temp)+(y_temp*y_temp);
		    end
		end
		
always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        resualt1 <= 7'b0;
		    end
		    else if(1) begin
						resualt1 <= resualt_temp1-arm_lenght+1;
				end
		end			
/***************************************************************斜边**********************************************************/
reg  [15:0]			  vertical_input2_temp;
reg  [7:0]			  resualt_temp2;
reg  [7:0]			  hypotenuse;

always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        vertical_input2_temp <= 16'b0;
		    end
		    else if(1) begin
		        vertical_input2_temp <= (short_side*short_side)+(resualt1*resualt1);
		    end
		end	

always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        hypotenuse <= 8'b0;
		    end
		    else if(1) begin
		        hypotenuse <= resualt_temp2+1 ;
		    end 
	end

/*****************************************************************************************************************************/
reg  [2:0]		sqrt_sel;
wire [15:0]	   sqrt_input;
reg  [15:0]		sqrt_input_temp;
wire [7:0]		sqrt_resualt;
wire				done;

always @(posedge clk or negedge rst_n) begin
	if(rst_n==1'b0)begin
			resualt_temp1 <= 8'b0;
			resualt_temp2 <= 8'b0;
			sqrt_input_temp<=16'b0;
			sqrt_sel<=3'b0;
		end
	else if(1)begin
    case (sqrt_sel)
		  3'b000:begin
					sqrt_input_temp<=vertical_input1_temp;
					sqrt_sel<=3'b001;
				end
		  3'b001:begin
					resualt_temp1<=sqrt_resualt;
					sqrt_sel<=3'b010;
				end
						
		  3'b010:begin
					sqrt_input_temp<=vertical_input2_temp;
					sqrt_sel<=3'b100;
				end
		  3'b100:begin
					resualt_temp2<=sqrt_resualt;
					sqrt_sel<=3'b000;
					end
        default: sqrt_sel<=3'b000;  
    endcase
	end
end	
	
assign sqrt_input=sqrt_input_temp;

sqrt sqrt(
	.radical(sqrt_input),		//输入
	.resualt(sqrt_resualt),				//整数部分输出
	.remainder(), 			//余数部分
	.done(done)
);		
/******************************************************短边********************************************************************/
reg  [15:0]			  short_side;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        short_side <= 0;
    end else begin
        // 合并条件判断
        short_side <= (base_Height >= z_temp) ? 
                      (base_Height - z_temp) : 
                      (z_temp - base_Height);
    end
end	
/***********************************************************大臂角*************************************************************/
reg signed [16:0] sum_temp; 

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sum_temp <= 16'd0;
    end 
	 else if(angle_flag==2'b01)begin
		sum_temp <= (arccos1 >>> 16) + (base_angle2 >>> 16)-90;
		end
	 else if(angle_flag==2'b10)begin
		sum_temp <= (arccos1 >>> 16) + (base_angle2 >>> 16);
		end
	 else begin
		sum_temp<=(arccos1 >>> 16);
		end
end

//assign  angle_y = sum_temp;
/********************************************************大臂角1***************************************************************/
reg  signed	[31:0] iData_temp1; 
reg signed [31:0] arccos1;   		// 反余弦结果（Q16格式）

always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        iData_temp1 <= 32'sd0;
		    end
		    else begin
			 iData_temp1 <= (hypotenuse*1638)>>2;
		    end
		end	

/***********************************************************小臂角*************************************************************/
reg  signed	[31:0] iData_temp2; 
reg  signed [31:0] small_arm_angle;		// 小臂角

//assign angle_x=(small_arm_angle>>16)+sum_temp-8'd90;
		
always  @(posedge clk or negedge rst_n)begin
		    if(rst_n==1'b0)begin
		        iData_temp2 <= 32'sd0;
		    end
			 
		    else begin
			 iData_temp2 <=32'd65536-((vertical_input2_temp*655)>>7);
		    end
		end
					
/*****************************************************************************************************************************/
reg  [1:0] 				arccos_sel;
wire signed[31:0]		arccos_resualt;
wire						arccos_vaild;
reg  signed[31:0]		arccos_input;

always @(posedge clk or negedge rst_n) begin
	if(rst_n==1'b0)begin
			arccos_input<=32'sd0;
			arccos1<=32'sd0;
			small_arm_angle<=32'sd0;
			arccos_sel<=2'b00;
		end
	else if(1)begin
    case (arccos_sel)
		  2'b00:begin
					arccos_input<=iData_temp1;
					arccos_sel<=2'b01;
				end
		  2'b01:begin
					if(arccos_vaild)begin
				   small_arm_angle<=arccos_resualt;
					arccos_sel<=2'b10;
					end
					else begin
					arccos_sel<=2'b01;
					end
				end			
		  2'b10:begin
					arccos_input<=iData_temp2;						
					arccos_sel<=2'b11;
				end
		  2'b11:begin
					if(arccos_vaild)begin
					arccos1<=arccos_resualt;
					arccos_sel<=2'b00;
					end
					else begin
					arccos_sel<=2'b11;						
					end
				end			
        default:	arccos_sel<=1'b0;		
     endcase
	end
end

cordic_arcsin_arccos arccos_cal(
      .clk(clk)        			 ,
      .rst_n(rst_n)      		 ,
      .iData(arccos_input)     ,   //16位小数位,-1~1  
      .pre_vaild(1)    			 ,
      .arcsin()      	       , 
      .arccos(arccos_resualt)  ,
      .post_vaild(arccos_vaild)
);	
/*****************************************************************************************************************************/

assign angle_z = base_angle1>>16;
assign  angle_y = sum_temp;
assign angle_x=(small_arm_angle>>16)+sum_temp-8'd90;

/*****************************************************************************************************************************/

endmodule 
