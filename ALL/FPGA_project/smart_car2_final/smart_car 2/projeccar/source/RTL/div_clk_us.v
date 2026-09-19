/**************
芯片晶振为50MHZ，HC_SR04需要一个10us的以上脉冲触发信号
所以这里我们需要对系统时钟进行分频，方便我们产生10us的持续电平
**************/
module div_clk_us (
    input sys_clk,
    input sys_rst_n,

    output wire  clk_us
);

//根据晶振换算，1us只需要计数50次即可

parameter [5:0] MAX_us = 6'd49;
reg [5:0] cnt;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n)begin
        cnt <= 6'd0;
    end
    else if(cnt == MAX_us)begin
        cnt <= 6'd0;
    end
    else begin
        cnt <= cnt + 6'd1;
    end
end
assign clk_us = cnt >= MAX_us;


endmodule