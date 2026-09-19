/****************
这个模块的主要功能就是对数据进行转码处理，
并给予串口发送模块使能
****************/
module uart_driver2 (
    input   wire            clk         ,
    input   wire            rstn        ,
    input   wire    [18:0]  data_in     , //测距数据
	input   wire            UART_rx     ,
    // input   wire            UART_rx     ,

    output  wire            UART_tx     
);
localparam   CLK_50MHz   =   26'd50_000_000    ;    // 时钟频率

wire            tx_done ;
wire            tx_falg; 

reg     [7:0]       data            ;
reg     [3:0]       xcnt            ;

reg		[3:0]	    cm_hund	        ;//100cm
reg		[3:0]	    cm_ten	        ;//10cm
reg		[3:0]	    cm_unit	        ;//1cm

reg		[3:0]	    point_1	        ;//1mm
reg		[3:0]	    point_2	        ;//0.1mm
reg		[3:0]	    point_3	        ;//0.01mm

reg     [25:0]      cnt_clk         ; 
wire                flag            ;

assign flag 	= cnt_clk == (CLK_50MHz/10);//每100ms使能一次进行发送，一次完整数据的发送刚好1s

 always @(posedge clk or negedge rstn) begin
     if(!rstn) begin
         cnt_clk <= 0;
     end
     else if(flag) begin
         cnt_clk <= 0;
     end
     else begin
         cnt_clk <= cnt_clk + 1;
     end
 end
//测距数据转换
always @(posedge clk or negedge rstn)begin  
	if(!rstn)begin  
		cm_hund	<= 'd0;
		cm_ten	<= 'd0;
		cm_unit	<= 'd0;

		point_1	<= 'd0;
		point_2	<= 'd0;
		point_3	<= 'd0;
	end  
	else begin  
		cm_hund <= data_in % 10;
		cm_ten	<= data_in / 10 ** 1 % 10;
		cm_unit <= data_in / 10 ** 2 % 10;

		point_1 <= data_in / 10 ** 3 % 10;
		point_2 <= data_in / 10 ** 4 % 10;
		point_3 <= data_in / 10 ** 5 % 10;
	end  
end 

//发送处理过程，在发送完一个数据后，计数加一，发送下一个数据
always@(posedge clk or negedge rstn) begin
    if(!rstn)
        xcnt <= 0;
    else if(tx_done)
        xcnt <= xcnt + 1'd1;
    else if(xcnt == 10)
        xcnt <= 0;
end
//根据计数器的值发送相应位置的数据
always @(*) begin
	case (xcnt)
		0    :    data = hex_data(point_3);
		1    :    data = hex_data(point_2);
		2    :    data = hex_data(point_1);
		3    :    data = "."              ;
		4    :    data = hex_data(cm_unit);
		5    :    data = hex_data(cm_ten) ;
		6    :    data = hex_data(cm_hund);
		7    :    data = "c"              ;
		8    :    data = "m"              ;
		9    :    data = "\n"             ;
		default:    data = 6'h30;
	endcase
end

//通过函数转化为ASCII码
// 函数，4位输入，8位输出，判断要输出的数字
function  [7:0]	hex_data; //函数不含时序逻辑相关
	input   [03:00]	data_i;//至少一个输入
	begin
		case(data_i)
			4'd0:hex_data = 8'h30;
			4'd1:hex_data = 8'h31;
			4'd2:hex_data = 8'h32;
			4'd3:hex_data = 8'h33;
			4'd4:hex_data = 8'h34;
			4'd5:hex_data = 8'h35;
			4'd6:hex_data = 8'h36;
			4'd7:hex_data = 8'h37;
			4'd8:hex_data = 8'h38;
			4'd9:hex_data = 8'h39;
			default:hex_data = 8'h30;
		endcase	
	end 
endfunction
//发送模块例化
uart_send uart_send_inst(
    /*input           */        .clk                   (clk ),
    /*input           */        .rstn                  (rstn),
    /*input           */        .flag_in              (flag),//发送使能
    /*input [7:0]     */        .data_in               (data),//待发送的距离数据,data

    // /*// output       */        .uart_tx_busy            (),//发送忙
    // /*// output       */        .en_flag                 (),
    // /*output reg      */        .tx_flag                 (tx_falg),//发送过程标志
    // /*output reg [7:0]*/        .tx_data                 (),//寄存发送数据
    // /*output reg [3:0]*/        .tx_cnt                  (),//发送数据计数器
    /*output wire     */        .tx_done                (tx_done),
    /*output reg      */        .UART_tx                (UART_tx)//发送端口
);
endmodule