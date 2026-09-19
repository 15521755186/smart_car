module data_handler#
(
    parameter [27:0] send_period = 28'd9_600_000
)
(
    input    wire    clk,
    input    wire    rst_n,

    input    wire    [7:0]   receive_command,
    input    wire    [7:0]   receive_flag,
    input    wire    tx_done_flag,
    
    
    input    wire    [7:0]   current_angle1 ,
    input    wire    [7:0]   current_angle2 ,
    input    wire    [7:0]   current_angle3 ,
    input    wire    [7:0]   current_angle4 ,

    input    wire    [1:0]   arm_op_state,
    input    wire    [1:0]   arm_state,

    input    wire    [18:0]  ultra_sonic_distance,

    input    wire    [7:0]    ir_data,

    input    wire    [7:0]    qr_code_data,

    input    wire    [7:0]    motor_state,
    input    wire    [7:0]    target_motor_speed1,
    input    wire    [7:0]    target_motor_speed2,
    input    wire    [7:0]    target_motor_speed3,
    input    wire    [7:0]    target_motor_speed4,
    input    wire    [3:0]    car_state,

    input    wire    transport_done_flag,
    
    output   reg     [7:0]     tx_data,
    output   reg     send_data_command

);

reg    [7:0]    pointer;
reg    [143:0]  data_stream;
reg    [7:0]    temp_data;
reg    [27:0]   send_period_cnt;
reg    load_flag;
reg    start_send_flag;
reg    send_flag;
reg    send_data_command_delay;

reg eg_detect1;
reg eg_detect2;

wire   tx_done;

assign tx_done = eg_detect1 & ~eg_detect2;

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)
            eg_detect1 <= 1'b0;
        else if (tx_done_flag == 1'b1)
            eg_detect1 <= 1'b1;
        else
            eg_detect1 <= 1'b0;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)
            eg_detect2 <= 1'b0;
        else if (eg_detect1 == 1'b1)
            eg_detect2 <= 1'b1;
        else
            eg_detect2 <= 1'b0;
    end


always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)
            send_period_cnt <= 28'd0;
        else if (send_period_cnt == send_period - 28'd1)
            send_period_cnt <=  28'd0;
        else
            send_period_cnt <= send_period_cnt + 28'd1;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)begin
            load_flag <= 1'b0;
            end
        else if (send_period_cnt == send_period - 28'd1000)
            load_flag <= 1'b1;
        else 
            load_flag <= 1'b0;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)
            send_data_command_delay <= 1'b0;
        else if(send_period_cnt == send_period - 28'd500)
            send_data_command_delay <= 1'b1;
        else if((tx_done == 1'b1) && (pointer !=8'd136))
            send_data_command_delay <= 1'b1;
        else
            send_data_command_delay <= 1'b0;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)
            send_data_command <= 1'b0;
        else if(send_data_command_delay == 1'b1)
            send_data_command <= 1'b1;
        else
            send_data_command <= 1'b0;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)
            pointer <= 8'd0;
        else if((pointer == 8'd136)&&(tx_done == 1'b1))
            pointer <= 8'd0;
        else if((tx_done == 1'b1))
            pointer <= pointer + 8'd08;
        else
            pointer <= pointer;
     end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)
            data_stream <= 137'd0;
        else if(load_flag) begin
        data_stream[15:0] <= 16'hffff;
        data_stream[47:16] <={current_angle1,current_angle2,current_angle3,current_angle4};
        data_stream[55:48] <={4'h0,arm_op_state,arm_state};
        data_stream[71:56] <= ultra_sonic_distance[18:3];
        data_stream[79:72] <= ir_data;
        data_stream[87:80] <= qr_code_data;
        data_stream[119:88] <= {target_motor_speed1,target_motor_speed2,target_motor_speed3,target_motor_speed4};
        data_stream[127:120] <={3'h0,transport_done_flag,car_state};
        data_stream[135:128] <= motor_state; end
        else
            data_stream <= data_stream;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)       
           tx_data <= 8'h00;
        else if(tx_done == 1'b1) begin
            case(pointer)  
             8'd0: tx_data <= data_stream[7:0];
             8'd8: tx_data <= data_stream[15:8];
             8'd16: tx_data <= data_stream[23:16];
             8'd24: tx_data <= data_stream[31:24];
             8'd32: tx_data <= data_stream[39:32];
             8'd40: tx_data <= data_stream[47:40];
             8'd48: tx_data <= data_stream[55:48];
             8'd56: tx_data <= data_stream[63:56];
             8'd64: tx_data <= data_stream[71:64];
             8'd72: tx_data <= data_stream[79:72];
             8'd80: tx_data <= data_stream[87:80];
             8'd88: tx_data <= data_stream[95:88];
             8'd96: tx_data <= data_stream[103:96];
             8'd104:tx_data <= data_stream[111:104];
             8'd112:tx_data <= data_stream[119:112];
             8'd120:tx_data <= data_stream[127:120];
             8'd128:tx_data <= data_stream[135:128];
//             8'd136:tx_data <= data_stream[143:136];
            default:tx_data<=8'h00;
           endcase end
        else
            tx_data <= tx_data;
   end
endmodule