`define START_0 3'b000
`define START_1 3'b001
`define START_2 3'b010
`define ALU_0 3'b011
`define WB_0 3'b100

module m_main(w_clk,w_rst,w_rrs,w_rrt,r_rslt);
    input wire w_clk,w_rst;
    input wire [31:0] w_rrs,w_rrt;
    output reg [31:0] r_rslt;

    reg [31:0] r_shiftrega = 0;
    reg [31:0] r_shiftregb = 0;
    reg[2:0] r_state = 0;
    reg[5:0] r_cnt = 0; 
    reg [1:0] r_carry = 0;
    reg [31:0] r_rrs,r_rrt;

    always@(posedge w_clk) begin
        if(w_rst) begin
            r_shiftrega <= 0;
            r_shiftregb <= 0;
        end else if(r_state==`START_0) begin
            r_state <= `START_1;
        end else if(r_state==`START_1) begin
            r_rrs <= w_rrs;
            r_rrt <= w_rrt;
            r_state <= `START_2;
        end else if(r_state==`START_2) begin    
            if(r_cnt[5]==0) begin
                r_shiftrega <= {r_rrs[0],r_shiftrega[31:1]};
                r_shiftregb <= {r_rrt[0],r_shiftregb[31:1]};
                r_rrs <= {1'b0,r_rrs[31:1]};
                r_rrt <= {1'b0,r_rrt[31:1]};
                r_cnt <= r_cnt + 1;
            end else begin
                r_state <= `ALU_0;
                r_cnt <= 0;
            end
        end else if(r_state==`ALU_0) begin
            if(r_cnt[5]==0) begin 
                {r_carry,r_shiftrega} <= {r_shiftrega[0] + r_shiftregb[0] + r_carry,r_shiftrega[31:1]};
                r_shiftregb <= {1'b0,r_shiftregb[31:1]};
                r_cnt <= r_cnt + 1;
            end else begin
                r_state <= `WB_0;
                r_cnt <= 0;
            end
        end else if(r_state==`WB_0) begin
            r_rslt <= r_shiftrega;
        end
    end

endmodule
