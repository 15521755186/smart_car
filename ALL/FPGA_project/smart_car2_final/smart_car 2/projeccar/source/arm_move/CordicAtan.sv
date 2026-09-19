module CordicAtan #(
    parameter DATA_WIDTH = 32,     // 输入数据宽度
    EXPAND_BIT = 16,               // 输入放大左移位数
    CYCLES     = 5                 // 迭代次数：2^CYCLES次
)(
    input                       clk,      // 时钟
    input                       rst_n,    // 异步复位
    
    input signed [DATA_WIDTH-1:0] x1,     // 输入坐标1-X
    input signed [DATA_WIDTH-1:0] y1,     // 输入坐标1-Y
    
    output [DATA_WIDTH-1:0] atan1,        // 结果1
    output                  valid1     // 结果1有效

);

reg [CYCLES-1:0] cnt_cycles1;
reg signed [DATA_WIDTH-1:0] x1_cycles, y1_cycles;
reg        [DATA_WIDTH-1:0] z1;
reg                         valid1_r;
wire        [DATA_WIDTH-1:0] z_w1;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cnt_cycles1 <= 5'b0;
    end 
	 else if(cnt_cycles1==5'b11111) begin
        cnt_cycles1 <=5'b0;
    end
	 else begin
		  cnt_cycles1 <= cnt_cycles1 + 1;
	 end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        valid1_r <= 1'b0;        // 复位时无效
    end 
    else if (cnt_cycles1 == 5'b11111) begin
        valid1_r <= 1'b1;        // 计满时置1
    end 
    else begin
        valid1_r <= 1'b0;        // 其他时刻保持0
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x1_cycles <= '0;
        y1_cycles <= '0;
        z1        <= 0;

    end else begin
        if (cnt_cycles1 == '0) begin
            case ({x1[DATA_WIDTH-1], y1[DATA_WIDTH-1]})
                2'b00: begin x1_cycles <= x1; y1_cycles <= y1; z1 <= 0; end
                2'b10: begin x1_cycles <= y1; y1_cycles <= -x1; z1 <= 90 << EXPAND_BIT; end
                2'b11: begin x1_cycles <= -x1; y1_cycles <= -y1; z1 <= 180 << EXPAND_BIT; end
                2'b01: begin x1_cycles <= -y1; y1_cycles <= x1; z1 <= 270 << EXPAND_BIT; end
            endcase

        end else begin
 
				 if (y1_cycles[DATA_WIDTH-1] == 0) begin
                x1_cycles <= x1_cycles + (y1_cycles >>> (cnt_cycles1-1));
                y1_cycles <= y1_cycles - (x1_cycles >>> (cnt_cycles1-1));
                z1        <= z1 + z_w1;
            end else begin
                x1_cycles <= x1_cycles + ((-y1_cycles) >>> (cnt_cycles1-1));
                y1_cycles <= y1_cycles + (x1_cycles >>> (cnt_cycles1-1));
                z1        <= z1 - z_w1;
            end
        end
    end
end

ROM_Atan ROM_Atan_inst1 (
    .addr          (cnt_cycles1),
    .clk           (clk),
    .rd_data       (z_w1)
);

assign valid1 = valid1_r;
assign atan1  = valid1_r ? z1 : 'z;

endmodule