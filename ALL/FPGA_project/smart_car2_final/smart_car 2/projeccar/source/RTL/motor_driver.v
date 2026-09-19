module motor_driver (
    input        clk,          // 绯荤粺鏃堕挓 (50MHz)
    input        rst_n,        // 浣庣數骞冲浣嶄俊鍙
    // 浜岀淮鐮佹帴鍙 
    input [7:0]  er_data,      // 浜岀淮鐮佹暟鎹
    input        er_flag,      // 浜岀淮鐮佹暟鎹湁鏁堟爣蹇
	     // 蓝牙控制接口（新增）
    input [7:0]  po_data,      // 蓝牙指令数据
    input        po_flag,      // 蓝牙指令有效标志
    // 鐏板害浼犳劅鍣
	     // 红外遥控
    input [7:0]  data,         // 红外解码键值 1：测试 2 ：二维码 3 ：追踪  4：蓝牙 5：避障
	     // 超声波测距
    input [18:0] data_o_r,     // 距离数据 (单位：mm)
    input        left_detector,  // 宸﹀惊杩逛紶鎰熷櫒 (1:妫€娴嬪埌榛戠嚎)
    input        right_detector, // 鍙冲惊杩逛紶鎰熷櫒 (1:妫€娴嬪埌榛戠嚎)
	 input        w_left_detector,
	 input        w_right_detector,
	 input        b_left_detector,
	 input        b_right_detector,
	 input        b_middle_detector,
    //机械臂完成状态输入
    input    [1:0]    arm_op_done,
    //机械臂状态输输出
    output   reg    [1:0]    arm_state,
    //仅做数据采集，应该不影响实际功能
    output   [7:0]    motor_state,
    output   [7:0]    target_motor_speed1,
    output   [7:0]    target_motor_speed2,
    output   [7:0]    target_motor_speed3,
    output   [7:0]    target_motor_speed4,
    output   [3:0]    car_state,
    output   reg      trans_done_flag,//完成运输标志位
    // 鐢垫満鎺у埗杈撳嚭
    output reg   AIN1, AIN2,    
    output reg   BIN1, BIN2,    
	 output reg   CIN1, CIN2,    
    output reg   DIN1, DIN2,    
    output       PWMA, PWMB, PWMC, PWMD     // 鐢垫満PWM璋冮€熶俊鍙
);
//----------------- PWM调速模块 -----------------//
reg [15:0] pwm_cnt;       // PWM计数器 (0-65535)
reg [7:0]  pwm_val_a;     // 电机A占空比 (0-255)
reg [7:0]  pwm_val_b;     // 电机B占空比 (0-255)
reg [7:0]  pwm_val_c;     // 电机A占空比 (0-255)
reg [7:0]  pwm_val_d;     // 电机B占空比 (0-255)
reg [3:0] mode;


// 安全距离参数 (单位：mm)
parameter SAFE_DISTANCE = 18'd15000;   // 15cm (避障模式阈值)
parameter TRACK_DISTANCE = 18'd10000;  // 10cm (跟随模式阈值)
parameter TRACK_PWM1 = 8'd45;  // 35
parameter TRACK_PWM2 = 8'd30;  //直线两轮速度差15  65  44
// PWM信号生成 (8位精度)
assign PWMA = (pwm_cnt[15:8] < pwm_val_a); // 比较计数器高8位与设定值
assign PWMB = (pwm_cnt[15:8] < pwm_val_b);
assign PWMC = (pwm_cnt[15:8] < pwm_val_c); // 比较计数器高8位与设定值
assign PWMD = (pwm_cnt[15:8] < pwm_val_d);

assign motor_state = {AIN1, AIN2, BIN1, BIN2, CIN1, CIN2, DIN1, DIN2};
assign target_motor_speed1 = pwm_val_a;
assign target_motor_speed2 = pwm_val_b;
assign target_motor_speed3 = pwm_val_c;
assign target_motor_speed4 = pwm_val_d;
assign car_state = state;
// PWM计数器累加
always @(posedge clk) begin
    pwm_cnt <= pwm_cnt + 1;
end
localparam IDLE           = 5'd0;
localparam ZHUA           = 5'd1;
localparam TRACE_GO       = 5'd2;
localparam GO_1S1         = 5'd3;
localparam turnr          = 5'd4; // 新增蓝牙控制状态
localparam turnl          = 5'd5;
localparam wan_TRACE      = 5'd6;
localparam FANG           = 5'd7; // 新增蓝牙控制状态
localparam TRACE_BACK     = 5'd8;
localparam turnr_b        = 5'd9;
localparam turnl_b        = 5'd10;
localparam BLUETOOTH_CTRL = 5'd15;
localparam GO_1S2         = 5'd12;
localparam wan_TRACE1     = 5'd13;
localparam GO_1S          = 5'd14; 
reg [4:0] state, next_state;
reg [1:0] COUNT;       
reg [27:0] timer;            
reg [27:0] timer1;
reg [27:0] timer2;
reg [1:0] flag;
reg timer_2s_done ; // 5绉掑畬鎴愭爣蹇
reg timer_0s8_done ; // 5绉掑畬鎴愭爣蹇
reg timer_0s5_done ; 
reg led;


// 计时器逻辑0.8s timer1
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) timer1 <= 0;
    else begin
        if (   (state == GO_1S)) timer1 <= timer1 + 1;
        else timer1 <= 0;
    end
end

//0s5 timer2
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) timer2 <= 0;
    else begin
        if ((state == turnl || state == turnr_b || state == turnl_b)) timer2 <= timer2 + 1;
        else timer2 <= 0;
    end
end




// 计时器逻辑2s  timer
always @(posedge clk or negedge rst_n) begin
   if (!rst_n) timer <= 0;
    else begin
       if (state == turnr) timer <= timer + 1;
        else timer <= 0;
    end
end



always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        timer_2s_done <= 0;
    end
    else begin
        if (timer >= 28'd20_000_000) begin
            timer_2s_done <= 1'b1;
        end
        else begin
            timer_2s_done <= 1'b0;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        timer_0s5_done <= 0;
    end
    else begin
        if (timer2 >= 28'd30_000_000) begin
            timer_0s5_done <= 1'b1;
        end
        else begin
            timer_0s5_done <= 1'b0;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin timer_0s8_done <= 0;  end
    else begin
        if (timer1 >= 28'd70_000_000) begin
		      timer_0s8_done <= 1'b1; 
		  end
		  else begin
		      timer_0s8_done <= 1'b0;
		  end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mode <= 4'b0000;
    end 
    else if (er_flag && (state == IDLE)) begin // 只在空闲状态接收新指令
        case(er_data)
            8'h31: begin mode <= 4'b0001; flag = 2'b00; end// 指令1
            8'h32: begin mode <= 4'b0010; flag = 2'b01; end// 指令2
			8'h33: begin mode <= 4'b0100; flag = 2'b10; end// 指令3
            8'h34: begin mode <= 4'b1000; flag = 2'b11; end// 指令4
        default: begin  mode <= 4'b0000; end // 其他指令复位模式
        endcase
    end
    // 完成任务后自动清除模式
    else if (state == IDLE) begin
        mode <= 4'b0000;
    end
end
// 鐘舵€佽浆绉
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= IDLE;
    else state <= next_state;
end

// 鐘舵€佹満缁勫悎閫昏緫
always @(*) begin
if (!rst_n) begin
next_state = IDLE;
trans_done_flag= 1'b0;
end
else begin
    next_state = state;
    case(state)
        IDLE:begin
		  if ((mode != 4'b0000) && (er_flag == 1'b1)) begin next_state =TRACE_GO;trans_done_flag= 1'b0; end 
		 // else  begin next_state = BLUETOOTH_CTRL; end
		  end 
		  ZHUA: 	begin if (arm_op_done == 2'b01)  next_state = TRACE_BACK;   end 
        TRACE_GO: begin
            if ({left_detector,w_left_detector, w_right_detector,right_detector}== 4'b1111) begin  next_state = GO_1S1; // 
        end
		  end

		  GO_1S1 :begin  
						if ((flag == 2'b00) && (COUNT == 2'd1)) begin
								next_state = turnl;//turnl
						end
						else if ((flag == 2'b01)&& (COUNT == 2'd1)) begin
								next_state = turnr;//turnr
						end
						 else if ((flag == 2'b10) && (COUNT == 2'd0)) begin
								next_state = GO_1S;
						end
						else if ((flag == 2'b11) && (COUNT == 2'd0)) begin
								next_state = GO_1S;
						end
                        else if ((flag == 2'b10) && (COUNT != 2'd0)) begin
								next_state = ZHUA;//turnl
						end
						else if ((flag == 2'b11) && (COUNT != 2'd0)) begin
								next_state = FANG;//turnr
						end
				      else begin
            // 四个条件均不满足时，进入TRACE_GO
                  next_state = TRACE_GO; 
                  end
					end
			turnr_b :begin if (timer_0s5_done == 1'b1) if(b_middle_detector==1'b1) next_state = wan_TRACE1;end 			
		    turnl :begin if (timer_0s5_done == 1'b1) if(b_left_detector==1'b1) next_state = wan_TRACE;end	
			wan_TRACE : if ({left_detector,w_left_detector, w_right_detector,right_detector}== 4'b0000) 
					next_state = ZHUA;
			wan_TRACE1 : if ({b_left_detector,b_middle_detector,b_right_detector}== 3'b000)begin 
					next_state = FANG;trans_done_flag = 1'b1;		end 
			FANG :  if  (arm_op_done == 2'b10) next_state = IDLE;
			
			TRACE_BACK: begin
             if (({left_detector,w_left_detector, w_right_detector,right_detector}== 4'b1111)&&(flag == 2'b00)) next_state = GO_1S2;
				else if (({left_detector,w_left_detector, w_right_detector,right_detector}== 4'b1111)&&(flag == 2'b01)) next_state = GO_1S2;
				else if (({left_detector,w_left_detector, w_right_detector,right_detector}== 4'b1111)&&(flag == 2'b10)) next_state = GO_1S2;
				else if (({left_detector,w_left_detector, w_right_detector,right_detector}== 4'b1111)&&(flag == 2'b11)) next_state = GO_1S2;
            else next_state = TRACE_BACK;
			/*	else if (({b_left_detector, b_middle_detector,b_right_detector}== 3'b111)&&(flag == 2'b01)&& (COUNT == 1)) next_state = turnl_b; 
				else if (({b_left_detector, b_middle_detector,b_right_detector}== 3'b111)&&(flag == 2'b10)&& (COUNT == 2)) next_state = turnr_b;
				else if (({b_left_detector, b_middle_detector,b_right_detector}== 3'b111)&&(flag == 2'b11)&& (COUNT == 2)) next_state = turnl_b;	*/		
            end
            GO_1S :begin
				if (timer_0s8_done == 1'b1) begin  
                next_state = TRACE_GO;     
                end
					end
			GO_1S2 :begin
				if ({left_detector,w_left_detector, w_right_detector,right_detector}== 4'b1111) begin  
						if (flag == 2'b00) begin
								next_state = turnr_b;
						end
						else if (flag == 2'b01) begin
								next_state = turnl_b;
						end
						else if (flag == 2'b10)begin
								next_state = wan_TRACE1;
						end
						else if (flag == 2'b11)begin
								next_state = wan_TRACE1;
						end
                  end
					end	
			turnl_b :begin if (timer_0s5_done == 1'b1) if(b_middle_detector==1'b1) next_state = wan_TRACE1;end 			
		    turnr :begin if (timer_2s_done == 1'b1) if(b_left_detector==1'b1) next_state = wan_TRACE;end		 
         BLUETOOTH_CTRL: if (data == 8'h46) 
                next_state = IDLE;		  
       endcase
		 end
end

// 鐢垫満鎺у埗閫昏緫
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
       {AIN1, AIN2, BIN1, BIN2} = 4'b0000;
		 {CIN1, CIN2, DIN1, DIN2} = 4'b0000;
        pwm_val_a = 8'd0;
        pwm_val_b = 8'd0;
		  pwm_val_c = 8'd0;
        pwm_val_d = 8'd0;
		  COUNT = 0;
		  arm_state = 2'b00;
    end else begin
        case(state)
				IDLE:begin
				    COUNT = 2'd0;
					{AIN1, AIN2, BIN1, BIN2} = 4'b0000;
					{CIN1, CIN2, DIN1, DIN2} = 4'b0000;
					pwm_val_a = 0;
					pwm_val_b = 0;
					pwm_val_c = 0;
					pwm_val_d = 0;
						end
				ZHUA: begin
				arm_state = 2'b01;
					{AIN1, AIN2, BIN1, BIN2} = 4'b0000;
					{CIN1, CIN2, DIN1, DIN2} = 4'b0000;
					pwm_val_a = 0;
					pwm_val_b = 0;
					pwm_val_c = 0;
					pwm_val_d = 0;
				     end
            TRACE_GO: begin  // 鍓嶅洓璺紶鎰熷櫒鎺у埗
                	case({w_left_detector,left_detector,right_detector,w_right_detector})
                    4'b1000: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2+8'd65;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2+8'd65;
                    end
                    4'b1100: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2+8'd35;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2+8'd35;
                    end
                   4'b1110: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2+8'd15;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2+8'd15;
                    end
                    4'b0110: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2;
                    end
                    4'b0001: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1+8'd65;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1+8'd65;
								pwm_val_d = TRACK_PWM2;
                    end
                    4'b0011: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1+8'd35;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1+8'd35;
								pwm_val_d = TRACK_PWM2;
                    end
                    4'b0111: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1+8'd15;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1+8'd15;
								pwm_val_d = TRACK_PWM2;
                    end   
                    4'b0010: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1+8'd15;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1+8'd15;
								pwm_val_d = TRACK_PWM2;
                    end                
                    4'b0100: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2+8'd15;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2+8'd15;
                    end 
                    4'b1111: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2;
                    end 
                default:begin
                              {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2;
                    end               
                endcase
            end
				GO_1S1: begin
				COUNT = COUNT +1'd1;
						{AIN1, AIN2, BIN1, BIN2} = 4'b0000; // 姝ｅ悜
						{CIN1, CIN2, DIN1, DIN2} = 4'b0000;
                        pwm_val_a = 20 ;  // 鍙宠疆閫熷害
                        pwm_val_b =20;  // 宸﹁疆閫熷害
								pwm_val_c = 20;
								pwm_val_d = 20;
				end
				turnr: begin
							{AIN1, AIN2, BIN1, BIN2} = 4'b1001;
							{CIN1, CIN2, DIN1, DIN2} = 4'b1001;
                        pwm_val_a = 8'd50;
                        pwm_val_b = 8'd50;
								pwm_val_c = 8'd130;
								pwm_val_d = 8'd130;
					     end
				turnl :begin
							{AIN1, AIN2, BIN1, BIN2} = 4'b0110;
							{CIN1, CIN2, DIN1, DIN2} = 4'b0110;
                                pwm_val_a = 8'd50;
                                pwm_val_b = 8'd50;
								pwm_val_c = 8'd120;
								pwm_val_d = 8'd120;
					     end
				wan_TRACE : begin
					case({w_left_detector,left_detector,right_detector,w_right_detector})
                    4'b1000: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2+8'd65;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2+8'd65;
                    end
                    4'b1100: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2+8'd35;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2+8'd35;
                    end
                   4'b1110: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2+8'd15;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2+8'd15;
                    end
                    4'b0110: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2;
                    end
                    4'b0001: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1+8'd65;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1+8'd65;
								pwm_val_d = TRACK_PWM2;
                    end
                    4'b0011: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1+8'd35;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1+8'd35;
								pwm_val_d = TRACK_PWM2;
                    end
                    4'b0111: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1+8'd15;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1+8'd15;
								pwm_val_d = TRACK_PWM2;
                    end   
                    4'b0010: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1+8'd15;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1+8'd15;
								pwm_val_d = TRACK_PWM2;
                    end                
                    4'b0100: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2+8'd15;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2+8'd15;
                    end 
                    4'b1111: begin // 00: 鍋滄
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2;
                    end 
                    default:begin
                              {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = TRACK_PWM1;
                                pwm_val_b = TRACK_PWM2;
								pwm_val_c = TRACK_PWM1;
								pwm_val_d = TRACK_PWM2;
                    end               
                endcase
				    end
					 wan_TRACE1 : begin
				    case({b_left_detector,b_middle_detector,b_right_detector})
                    3'b111: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2;
                    end
                    3'b110: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1+8'd35;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1+8'd35;
						pwm_val_d = TRACK_PWM2;
                    end
                     3'b100: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1+8'd65;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1+8'd65;
						pwm_val_d = TRACK_PWM2;
                    end
                     3'b001: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2+8'd65;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2+8'd65;
                    end
                     3'b011: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2+8'd35;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2+8'd35;

                    end 
                    3'b010: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2;

                    end  
                    default :begin  
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2;
                    end                       
                endcase
				    end
				FANG : begin
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0000;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0000;
                        pwm_val_a = 8'd0;
                        pwm_val_b = 8'd0;
						pwm_val_c = 8'd0;
						pwm_val_d = 8'd0;
				    arm_state = 2'b10;
				end
				TRACE_BACK: begin // 鍚庡洓璺紶鎰熷櫒鎺у埗
                     case({b_left_detector,b_middle_detector,b_right_detector})
                    3'b111: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2;
                    end
                    3'b110: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1+8'd35;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1+8'd35;
						pwm_val_d = TRACK_PWM2;
                    end
                     3'b100: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1+8'd65;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1+8'd65;
						pwm_val_d = TRACK_PWM2;
                    end
                     3'b001: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2+8'd65;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2+8'd65;
                    end
                     3'b011: begin // 00: 鍋滄
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2+8'd35;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2+8'd35;

                    end  
                    default :begin  
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0101;
						{CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = TRACK_PWM1;
                        pwm_val_b = TRACK_PWM2;
						pwm_val_c = TRACK_PWM1;
						pwm_val_d = TRACK_PWM2;
                    end                        
                endcase
						  end
				GO_1S2: begin
						{AIN1, AIN2, BIN1, BIN2} = 4'b1010; // 姝ｅ悜
						{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                        pwm_val_a = 8'd25;  // 鍙宠疆閫熷害
                        pwm_val_b = 8'd38;  // 宸﹁疆閫熷害
								pwm_val_c = 8'd25;
								pwm_val_d = 8'd38;
				end
                GO_1S: begin
						{AIN1, AIN2, BIN1, BIN2} = 4'b1010; // 姝ｅ悜
						{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                        pwm_val_a = 8'd30;  // 鍙宠疆閫熷害
                        pwm_val_b = 8'd30;  // 宸﹁疆閫熷害
								pwm_val_c = 8'd30;
								pwm_val_d = 8'd30;
				end
				turnr_b: begin
							{AIN1, AIN2, BIN1, BIN2} = 4'b1001;
							{CIN1, CIN2, DIN1, DIN2} = 4'b1001;
                                pwm_val_a = 8'd50;
                                pwm_val_b = 8'd50;
								pwm_val_c = 8'd150;
								pwm_val_d = 8'd150;				
					     end
				turnl_b :begin
							{AIN1, AIN2, BIN1, BIN2} = 4'b0110;
							{CIN1, CIN2, DIN1, DIN2} = 4'b0110;
                        pwm_val_a = 8'd50;
                        pwm_val_b = 8'd50;
								pwm_val_c = 8'd145;
								pwm_val_d = 8'd145;				
					     end			  
				BLUETOOTH_CTRL: begin  // 新增蓝牙控制状态
                case(er_data)
                    8'h31: begin // 'F' 前进 快速左转
                                {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                                pwm_val_a = 8'd40;
                                pwm_val_b = 8'd40;
								pwm_val_c = 8'd40;
								pwm_val_d = 8'd40;
                    end
                    8'h32: begin // 'B' 后退 稳定左转3////////
                        {AIN1, AIN2, BIN1, BIN2} = 4'b1001;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1001;
                                pwm_val_a = 8'd50;
                                pwm_val_b = 8'd50;
								pwm_val_c = 8'd100;
								pwm_val_d = 8'd100;
                    end
                    8'h33: begin // 原地转
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0110;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1001;
                                pwm_val_a = 8'd50;
                                pwm_val_b = 8'd50;
								pwm_val_c = 8'd100;
								pwm_val_d = 8'd100;
                    end
                    8'h34: begin // 'R' 右转
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0110;
								{CIN1, CIN2, DIN1, DIN2} = 4'b1001;
                                pwm_val_a = 8'd50;
                                pwm_val_b = 8'd50;
								pwm_val_c = 8'd110;
								pwm_val_d = 8'd110;
                    end
                    8'h07: begin arm_state = 2'b01; end
                    8'h15: begin {AIN1, AIN2, BIN1, BIN2} = 4'b0110;
								{CIN1, CIN2, DIN1, DIN2} = 4'b0110;
                                pwm_val_a = 8'd50;
                                pwm_val_b = 8'd50;
								pwm_val_c = 8'd130;
								pwm_val_d = 8'd130; end
                    8'h09: begin arm_state = 2'b10; end //用蓝牙试过可以输入状态，但红外因为不知道键码多少加上难操作，没试成功，但应该是可以用的
						  8'h1C: begin // OK键：停止
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0000;
								{CIN1, CIN2, DIN1, DIN2} = 4'b0000;
                        pwm_val_a = 8'd0;
                        pwm_val_b = 8'd0;
                    end
						   8'h44: begin
						  case(po_data)
                        8'h41: begin // '1': 前进
                            {AIN1, AIN2, BIN1, BIN2} = 4'b1010; // 双正转
									 {CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                        pwm_val_a = 8'd80;
                        pwm_val_b = 8'd80;
								pwm_val_c = 8'd80;
								pwm_val_d = 8'd80;
                        end
                        8'h42: begin // '2': 后退
                            {AIN1, AIN2, BIN1, BIN2} = 4'b0101; // 双反转
									 {CIN1, CIN2, DIN1, DIN2} = 4'b0101;
                        pwm_val_a = 8'd80;
                        pwm_val_b = 8'd80;
								pwm_val_c = 8'd80;
								pwm_val_d = 8'd80;
                        end
                        8'h40: begin // '0': 停止
                            {AIN1, AIN2, BIN1, BIN2} = 4'b0000;
									 {CIN1, CIN2, DIN1, DIN2} = 4'b0000;
                            pwm_val_a = 8'd0;
                            pwm_val_b = 8'd0;
                        end
								endcase
						  end 
						   8'h40: begin
						  //避障
						  if(data_o_r < SAFE_DISTANCE) begin // 距离小于15cm
                    {AIN1, AIN2, BIN1, BIN2} = 4'b0110; // 左转避障
						  {CIN1, CIN2, DIN1, DIN2} = 4'b0110;
                    pwm_val_a = 8'd80;
                    pwm_val_b = 8'd80;
						  pwm_val_c = 8'd80;
                    pwm_val_d = 8'd80;
                end
                else begin // 安全距离直行
                    {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
						  {CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                    pwm_val_a = 8'd80;
                    pwm_val_b = 8'd80;
						  pwm_val_c = 8'd80;
                    pwm_val_d = 8'd80;
                end
						  end 
						   8'h47: begin
							if(data_o_r < SAFE_DISTANCE) begin // 距离小于15cm
                    {AIN1, AIN2, BIN1, BIN2} = 4'b1010; // 左转避障
					{CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                    pwm_val_a = 8'd40;
                    pwm_val_b = 8'd40;
						  pwm_val_c = 8'd40;
                    pwm_val_d = 8'd40;
						  
                end
                else begin // 安全距离直行
                    {AIN1, AIN2, BIN1, BIN2} = 4'b1010;
						  {CIN1, CIN2, DIN1, DIN2} = 4'b1010;
                    pwm_val_a = 8'd80;
                    pwm_val_b = 8'd80;
				        pwm_val_c = 8'd80;
                    pwm_val_d = 8'd80;
                end
					 end
                    default: begin // 停止
                        {AIN1, AIN2, BIN1, BIN2} = 4'b0000;
								{CIN1, CIN2, DIN1, DIN2} = 4'b0000;
                        pwm_val_a = 8'd0;
                        pwm_val_b = 8'd0;
								pwm_val_c = 8'd0;
								pwm_val_d = 8'd0;
                    end
                endcase
            end		  
endcase
end
end

endmodule 
