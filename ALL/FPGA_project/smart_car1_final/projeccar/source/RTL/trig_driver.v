/****************
根据分频的1us时钟，产生一个持续10us的电平用于驱动HC_SR04
最好是稍微大于10us，这样稳妥一些
****************/
module trig_driver(
    input       sys_us        ,//1us时钟
    input       sys_rst_n     ,

    output      trig          //驱动超声波的信号
);

parameter T = 19'd29_9999;//设置触发信号的周期，这里设置得越小，其触发越频繁，应该返回的距离更新更频繁

reg [18:0] cnt;

always @(posedge sys_us) begin// or negedge sys_rst_n
    if(!sys_rst_n)begin
        cnt <= 19'd0;
    end
    else if(cnt == T)begin
        cnt <= 19'd0;
    end
    else begin
        cnt <= cnt + 1'd1;
    end
end
//15us的高电平
assign trig = (cnt <15 ) ? 1'b1 : 1'b0;//正确的，只是时间太短，观察不到，目前应该是串口问题
endmodule