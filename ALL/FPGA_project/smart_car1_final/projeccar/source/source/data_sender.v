module data_sender
(
    input    wire    clk,
    input    wire    serial_port_clk,
    input    wire    rst_n,
    input    wire    send_command, 

    input    wire    [7:0]    input_data,
    //output   reg    state_sending,
    output   reg      send_data,
    output   wire     start_flag,
    output   reg    stop_flag
    //output   reg    send_command_edge_detect1
);

reg    [0:4]    index;

reg    serial_port_clk_edge_detect1;
reg    serial_port_clk_edge_detect2;

reg    send_command_edge_detect1;
reg    send_command_edge_detect2;

wire   start_flag;
//reg    stop_flag;

wire   send_flag;

reg    state_sending;

assign  send_flag = ~serial_port_clk_edge_detect1 & serial_port_clk_edge_detect2;
assign  start_flag = send_command_edge_detect1 & ~send_command_edge_detect2;

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            serial_port_clk_edge_detect1 <= 1'b0;
        else if (serial_port_clk == 1'b1)
            serial_port_clk_edge_detect1 <= 1'b1;
        else
            serial_port_clk_edge_detect1 <= 1'b0;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            serial_port_clk_edge_detect2 <= 1'b0;
        else if (serial_port_clk_edge_detect1 == 1'b1)
            serial_port_clk_edge_detect2 <= 1'b1;
        else
            serial_port_clk_edge_detect2 <= 1'b0;
    end


always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            send_command_edge_detect1 <= 1'b0;
        else if (send_command == 1'b1)
            send_command_edge_detect1 <= 1'b1;
        else
            send_command_edge_detect1 <= 1'b0;
    end

always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            send_command_edge_detect2 <= 1'b0;
        else if (send_command_edge_detect1 == 1'b1)
            send_command_edge_detect2 <= 1'b1;
        else
            send_command_edge_detect2 <= 1'b0;
    end


always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'd0)
            state_sending <= 1'b0;
        else if (state_sending == 1'b1)
            begin
                if (stop_flag == 1'b1)
                    state_sending <= 1'b0;
                else
                    state_sending <= state_sending;
            end
        else if (start_flag == 1'b1)
            state_sending <= 1'b1;
        else
            state_sending <= 1'b0;
    end


always@(posedge clk or negedge rst_n)
    begin
        if(rst_n==1'b0)
            begin index <= 4'b0000; send_data <= 1'b1; stop_flag <= 1'b0; end
        else if((state_sending == 1'b1 )&& (send_flag == 1'b1))
            begin
            case(index)
                4'b0000: begin send_data <= 1'b0; index <= 4'b0001; end        
                4'b0001: begin send_data <= input_data[0]; index <= 4'b0010; end
                4'b0010: begin send_data <= input_data[1]; index <= 4'b0011; end
                4'b0011: begin send_data <= input_data[2]; index <= 4'b0100; end
                4'b0100: begin send_data <= input_data[3]; index <= 4'b0101; end
                4'b0101: begin send_data <= input_data[4]; index <= 4'b0110; end
                4'b0110: begin send_data <= input_data[5]; index <= 4'b0111; end
                4'b0111: begin send_data <= input_data[6]; index <= 4'b1000; end
                4'b1000: begin send_data <= input_data[7]; index <= 4'b1001; end
                4'b1001: begin send_data <= 1'b1; index <= 4'b0000; stop_flag <= 1'b1; end
                //4'b1010: begin send_data <= 1'b1; index <= 4'b0000;  end
                default: begin send_data <= 1'b1; index <= 4'b0000; stop_flag <= 1'b0; end        
            endcase
            end
        else if (stop_flag == 1'b1)
            stop_flag <=1'b0;  
        else if ((state_sending == 1'b1) && (send_flag == 1'b0))
            begin send_data <= send_data; index <= index; end
        else
            begin index <= 4'b0000; send_data <= 1'b1; stop_flag <=1'b0; end
    end
    
endmodule
