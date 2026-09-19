module receiver#
(
    parameter    bit_rate   =    10'd1000
)
(
    input    wire    rst_n,
    input    wire    clk,
    input    wire    receive_data,

    output   reg     [7:0]   data,
    output   reg     [7:0]   copy_data,
    output   reg     end_of_recieve_flag,
    output   reg     clk_div

);

reg    [0:3]    state;   
reg    recieve_flag;
reg    [7:0]    temp_data;
reg    [0:11]   clk_div_cnt;
reg    clk_div_flag;
reg    clk_div;


always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            clk_div_cnt <= 1'd0;
        else if(clk_div_cnt == (bit_rate - 1'd1))
            clk_div_cnt <= 1'd0;
        else
            clk_div_cnt <= clk_div_cnt + 1'd1;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            clk_div_flag <= 1'd0;
        else if (clk_div_cnt == (bit_rate - 2'd2))
            clk_div_flag <= 1'd1;
        else
            clk_div_flag <= 1'd0;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            recieve_flag <= 1'd0;
        else if ((clk_div_cnt == (bit_rate >> 1)) && (clk_div == 1'b1))
            recieve_flag <= 1'd1;
        else
            recieve_flag <= 1'd0;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            clk_div <= 1'd0;
        else if (clk_div_flag == 1'd1)
            clk_div <= ~clk_div;
        else
            clk_div <= clk_div;
    end



always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            begin state <= 4'b0000; temp_data <= 8'h00; data <= 8'h00; end_of_recieve_flag <= 1'd0; end
        else if(recieve_flag == 1'b1)
            begin
            case(state)
                4'b0000: 
                begin
                    if (receive_data == 1'b0)
                        begin
                            state <= 4'b0001;
                            end_of_recieve_flag <= 1'd0;
                        end
                    else
                        begin
                            state <= 4'b0000 ;
                            end_of_recieve_flag <= 1'd0;
                        end
                end
                4'b0001: begin temp_data[0] <= receive_data;    state <= 4'b0010;   end
                4'b0010: begin temp_data[1] <= receive_data;    state <= 4'b0011;   end
                4'b0011: begin temp_data[2] <= receive_data;    state <= 4'b0100;   end
                4'b0100: begin temp_data[3] <= receive_data;    state <= 4'b0101;   end
                4'b0101: begin temp_data[4] <= receive_data;    state <= 4'b0110;   end
                4'b0110: begin temp_data[5] <= receive_data;    state <= 4'b0111;   end
                4'b0111: begin temp_data[6] <= receive_data;    state <= 4'b1000;   end
                4'b1000: begin temp_data[7] <= receive_data;    state <= 4'b1001;   end
                4'b1001: 
                begin
                    if (receive_data == 1'd1)
                        begin
                            data <= temp_data;
                            copy_data <= temp_data;
                            end_of_recieve_flag <= 1'd1;
                            state <= 4'b0000;
                        end
                    else
                        begin
                            data <= data;
                            copy_data <= copy_data;
                            end_of_recieve_flag <= 1'd1;
                            state <= 4'b0000;
                            temp_data <= 4'b0000;
                        end
                 end
                default: begin state <= 4'b0000; temp_data <= 8'h00; data <= 8'h00; copy_data <= 8'h00; end_of_recieve_flag <= 1'd0; end
            endcase
            end
        else
            begin state <= state; temp_data <= temp_data; data <= data; copy_data <=copy_data; end_of_recieve_flag <= end_of_recieve_flag; end
    end

endmodule 

