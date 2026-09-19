module top
(
    input    wire    clk,
    input    wire    clk_27m,
    input    wire    sys_rst_n,
    input    wire    receive_data,

     input        inf_in,         // ���������ź�
	 input        bluetooth_rx,   // �����ڽ���
	 input        scanner_rx,     // 二维码扫描器串口接收  
	 input        echo,           // �������ز��ź�

    input        left_detector,  // 左循迹传感器 (1:棢�测到黑线)
    input        right_detector, // 右循迹传感器 (1:棢�测到黑线)
	 input        w_left_detector,
	 input        w_right_detector,
	 input        b_left_detector,
	 input        b_right_detector,
	 input        b_middle_detector,
    //input    wire    key,

    output   wire    clk_19_2M,
    output   wire    send_data,
//    output   wire    state_sending,
    output   wire    [7:0]    data,
//    output   wire    start_flag,
//    output   wire    stop_flag,
//    output   wire    send_command_edge_detect1,
//    output   wire    bridge_trigger
//    output    reg    test

	 output       trig,           // �����������ź�
    output       uart_tx,       // ���������Դ�������
    output    AIN1, AIN2,    // 电机A方向控制
    output    BIN1, BIN2,    // 电机B方向控制
	 output    CIN1, CIN2,    // 电机A方向控制
    output    DIN1, DIN2,    // 电机B方向控制
    output       PWMA, PWMB, PWMC, PWMD,    // PWM输出
    output       led_mode,           // 状����指示灯

    output   wire    gear1,
    output   wire    gear2,
    output   wire    gear3,
    output   wire    gear4

);

wire    pll_lock;

wire    [7:0]    bridge_data;
wire    bridge_trigger;
wire    bridge_clk;
wire    start_flag;
wire    stop_flag;
wire    [7:0]    tx_data;
wire    send_data_command;


wire    [1:0]    done;
wire    [1:0]	 flag;
wire    [1:0]    state_input;
wire    [15:0]   x_input;
wire    [15:0]   y_input;
wire    [15:0]   z_input;

wire    [7:0]    angle1_input;
wire    [7:0]    angle2_input;
wire    [7:0]    angle4_input;

wire    [7:0]    servo_angle_in1;
wire    [7:0]    servo_angle_in2;
wire    [7:0]    servo_angle_in3;
wire    [7:0]    servo_angle_in4;

wire    ack1;
wire    ack2;
wire    ack3;
wire    ack4;

wire    [7:0]motor_state        ;
wire    [7:0]target_motor_speed1;
wire    [7:0]target_motor_speed2;
wire    [7:0]target_motor_speed3;
wire    [7:0]target_motor_speed4;
wire    [3:0]car_state          ;
wire    trans_done_flag;



parameter   UART_BPS    =   14'd9600        ,   //比特��         
            CLK_FREQ    =   26'd50_000_000  ;   //时钟频率
//----------------- ������ģ�� ----------------//
wire       clk_us;       // 1MHzʱ��
wire [18:0] data_o_r;    // ����������ݣ���λ��mm��
//wire  define
wire    [7:0]   er_data;
wire            er_flag;
wire    [7:0]   po_data;
wire            po_flag;
//----------------- ����ң�� -----------------//
wire [7:0] inf_data;         // ������������
reg   led;        

bluetooth_clk_generator u_bluetooth_clk_generator
(
    .clkin1        (clk_27m), 
    .clkout0       (clk_19_2M),
    .pll_lock      (pll_lock)
);

receiver u_receiver
(
    .rst_n        (sys_rst_n),
    .clk          (clk_19_2M),
    .receive_data (receive_data),

    .copy_data                (er_data),
    .end_of_recieve_flag      (er_flag),
    .clk_div                  (bridge_clk),
    .data                     (data)
);

data_sender u_data_sender
(
    .send_data          (send_data),
    .clk                (clk_19_2M),
    .rst_n              (sys_rst_n),

    .serial_port_clk    (bridge_clk),
    .send_command       (bridge_trigger),
    .input_data         (bridge_data),
    .start_flag         (start_flag),
    .stop_flag          (stop_flag)
   // .state_sending      (state_sending),
   // .send_command_edge_detect1 (send_command_edge_detect1)
);

mian machine_arm_inst(	
    .sys_clk        (clk),
    .rst_n          (sys_rst_n),
    .done           (done),
    .state_input    (state_input),
    .x              (x_input),
    .y              (y_input),
    .z              (z_input),

    .flag           (flag)
);

calculate angle_calculate_inst(
    .clk            (clk),
    .rst_n          (sys_rst_n),

    .x              (x_input),
    .y              (y_input),
    .z              (z_input),
    
    .angle_x        (angle1_input),
    .angle_y        (angle4_input),
    .angle_z        (angle2_input)
);

 servo_ctr servo_ctr_inst(	
	.sys_clk		(clk),
	.rst_n		    (sys_rst_n),
	.flag			(flag),
	.angle1_in  (angle1_input),
	.angle2_in  (angle2_input),
	.angle4_in  (angle4_input),
	.done		(done),

    .ack1        (ack1),
    .ack2        (ack2),
    .ack3        (ack3),
    .ack4        (ack4),

    .angle1      (servo_angle_in1),
    .angle2      (servo_angle_in2),
    .angle3      (servo_angle_in3),
    .angle4      (servo_angle_in4)

//	.gear1		(gear1),
//	.gear2		(gear2),
//	.gear3		(gear3),
//	.gear4		(gear4)
);

servo	servo1(
	.sys_clk		(clk),
	.rst_n			(sys_rst_n),	
	.gear_req		(1'b1),				//����ת������
	.angle			(servo_angle_in1),					//ת���Ƕ�
	.gear_ack		(ack1),				//����ת������
	.gear			(gear1)
);

servo	servo2(
	.sys_clk		(clk),
	.rst_n			(sys_rst_n),	
	.gear_req		(1'b1),				//����ת������
	.angle			(servo_angle_in2),					//ת���Ƕ�
	.gear_ack		(ack2),				//����ת������
	.gear			(gear2)
);

servo	servo3(
	.sys_clk		(clk),
	.rst_n			(sys_rst_n),	
	.gear_req		(1'b1),				//����ת������
	.angle			(servo_angle_in3),					//ת���Ƕ�
	.gear_ack		(ack3),				//����ת������
	.gear			(gear3)
);

servo	servo4(
	.sys_clk		(clk),
	.rst_n			(sys_rst_n),	
	.gear_req		(1'b1),				//����ת������
	.angle			(servo_angle_in4),					//ת���Ƕ�
	.gear_ack		(ack4),				//����ת������
	.gear			(gear4)
);

div_clk_us div_clk_us_inst(
    .sys_clk     (clk),
    .sys_rst_n   (sys_rst_n),
    .clk_us      (clk_us)
);

// �����������ź�����
trig_driver trig_driver_inst(
    .sys_us      (clk_us),  // 1MHzʱ��
    .sys_rst_n   (sys_rst_n),
    .trig        (trig)     // �����ź�
);

// �������ز�����
echo_driver echo_driver_inst(
    .sys_clk     (clk),
    .sys_us      (clk_us),
    .sys_rst_n   (sys_rst_n),
    .echo        (echo),
    .data_o      (data_o_r) // ������룬��λmm
);

// ���������ݴ�������
uart_driver2 uart_driver2_inst(
    .clk         (clk),
    .rstn        (sys_rst_n),
    .data_in     (data_o_r),
    .UART_tx     (uart_tx)
);

uart_rx
#(
    .UART_BPS    (UART_BPS  ),  //串口波特��   
	 .CLK_FREQ    (CLK_FREQ  )   //时钟频率
)
uart_rx_inst
(
    .sys_clk     (clk),
    .sys_rst_n   (sys_rst_n),
    .rx          (scanner_rx)
    //.po_data     (er_data),
    //.po_flag     (er_flag)
);

// ������ģ��
uart_rx #(
    .UART_BPS(9600),
    .CLK_FREQ(50_000_000)
) uart_rx_inst1(
    .sys_clk     (clk),
    .sys_rst_n   (sys_rst_n),
    .rx          (bluetooth_rx),
    .po_data     (po_data),
    .po_flag     (po_flag)
);
// ��������ģ��
inf_rcv u_inf_rcv(
    .clk         (clk),
    .rst_n       (sys_rst_n),
    .inf_in      (inf_in),
    .data        (inf_data)  // �������İ���ֵ
);


// ��������ģ��
motor_driver u_motor
(
    .clk            (clk),
    .rst_n          (sys_rst_n),
    .er_data        (er_data),     // ��ά������
    .er_flag        (er_flag),     // ��ά��������Ч��־
	 .po_data        (po_data),     // ��ά������
    .po_flag        (po_flag),     // ��ά��������Ч��־
	 .data           (inf_data),        // ��������
	 .data_o_r       (data_o_r),    // ��������������
    .left_detector   (left_detector),  // ��ѭ�������� (1:���⵽����)
    .right_detector  (right_detector), // ��ѭ�������� (1:���⵽����)
	 .w_left_detector  (w_left_detector),
	 .w_right_detector  (w_right_detector),
	 .b_left_detector  (b_left_detector),
	 .b_right_detector  (b_right_detector),
	 .b_middle_detector  (b_middle_detector),

    .arm_op_done    (done),
    .arm_state      (state_input),

    .motor_state              (motor_state        ),
    .target_motor_speed1      (target_motor_speed1),
    .target_motor_speed2      (target_motor_speed2),
    .target_motor_speed3      (target_motor_speed3),
    .target_motor_speed4      (target_motor_speed4),
    .car_state                (car_state          ),
    .trans_done_flag          (trans_done_flag),

    .AIN1       (AIN1),
    .AIN2       (AIN2),
    .BIN1       (BIN1),
    .BIN2       (BIN2),
	.CIN1       (CIN1),
    .CIN2       (CIN2),
    .DIN1       (DIN1),
    .DIN2       (DIN2),
    .PWMA           (PWMA),
    .PWMB           (PWMB),
	.PWMC           (PWMC),
    .PWMD           (PWMD)
);

data_handler inst_data_handler
(
    .clk                      (clk),
    .rst_n                    (sys_rst_n),
                             
    .receive_command          (bridge_data),
    .receive_flag             (bridge_trigger),
    .tx_done_flag             (stop_flag),
                            
    .current_angle1           (servo_angle_in1),
    .current_angle2           (servo_angle_in2),
    .current_angle3           (servo_angle_in3),
    .current_angle4           (servo_angle_in4),
                            
    .arm_op_state             (done),
    .arm_state                (state_input),
                             
    .ultra_sonic_distance     (data_o_r),
                              
    .ir_data                  (inf_data),
                              
    .qr_code_data             (er_data),
                              
    .motor_state              (motor_state        ),
    .target_motor_speed1      (target_motor_speed1),
    .target_motor_speed2      (target_motor_speed2),
    .target_motor_speed3      (target_motor_speed3),
    .target_motor_speed4      (target_motor_speed4),
    .car_state                (car_state          ),

    .transport_done_flag      (trans_done_flag),
                              
    .tx_data                  (bridge_data),
    .send_data_command        (bridge_trigger)

);

always @(posedge clk or negedge sys_rst_n) begin
    if(!sys_rst_n) 
        led <= 1'b0;  // 灯灭
    //蓝牙控车模式 灯灭
   else if(er_flag && er_data == 8'h31) // ASCII 'a'
        led <= 1'b1;//
	else if(er_flag && er_data == 8'h32) // ASCII 'a'
        led <= 1'b0;//
	else if(po_flag && po_data == 8'h31) // ASCII 'a'
        led <= 1'b1;//
	else if(po_flag && po_data == 8'h32) // ASCII 'a'
        led <= 1'b0;//	
	else if(inf_data == 8'h07) // ���ⰴ��1
        led <= 1'b1;// 	  
	else if(inf_data == 8'h09) // ���ⰴ��5
        led <= 1'b0;//  
end



assign led_mode = led;
endmodule
