`include "config.vh"

module m_main(w_clk,w_rst,r_rslt);
    //define input output
    input wire w_clk,w_rst;
    output reg [31:0] r_rslt;

    reg [2:0] r_state = 0;
    reg [5:0] r_cnt = 1; 
    reg [31:0] r_pc,r_npc; //program counter
    reg [31:0] r_rrs1,r_rrs2; //value of rs1 rs2
    reg [31:0] r_shiftrega = 0;
    reg [31:0] r_shiftregb = 0;
    reg [1:0] r_carry = 0;

/************************  START_0  ************************/
    wire [31:0] w_ir;
    wire [4:0] w_rs1,w_rs2,w_rd; //address of operand
    wire [6:0] w_funct7;
    wire [2:0] w_funct3;
    wire [6:0] w_opcode;
    wire [31:0] w_rrs1,w_rrs2;
    wire [3:0] w_ctl; //controller of ALU
    wire w_we;

    m_memory imem(w_clk, r_pc[12:2] , 0 , 0 , w_ir);

    assign w_opcode = w_ir[6:0];
    assign w_rd = w_ir[11:7];
    assign w_funct3 = w_ir[14:12];
    assign w_rs1 = w_ir[19:15];
    assign w_rs2 = w_ir[24:20];
    assign w_funct7 = w_ir[31:25];

    m_regfile regfile(w_clk, w_rs1, w_rs2, w_rd, w_we, w_wdata, w_rrs1, w_rrs2);

    always@(posedge w_clk) begin
        if (r_state==`START_0) begin
            r_pc <= r_pc + 4;
            r_rrs1 <= w_rrs1;
            r_rrs2 <= w_rrs2;
            r_state <= `START_1;
        end
    end

/************************  START_1  ************************/

    always@(posedge w_clk) begin
        if(r_state==START_1) begin
            if(r_cnt[5]==0) begin
                r_shiftrega <= {r_rrs[0],r_shiftrega[31:1]};
                r_shiftregb <= {r_rrt[0],r_shiftregb[31:1]};
                r_rrs <= {1'b0,r_rrs[31:1]};
                r_rrt <= {1'b0,r_rrt[31:1]};
                r_cnt <= r_cnt + 1;
            end else begin
                r_shiftrega <= {r_rrs[0],r_shiftrega[31:1]};
                r_shiftregb <= {r_rrt[0],r_shiftregb[31:1]};
                r_rrs <= {1'b0,r_rrs[31:1]};
                r_rrt <= {1'b0,r_rrt[31:1]};
                r_state <= `START_2;
                r_cnt <= 1;
            end
        end
    end

/************************  START_2(to determine next state)  ************************/
    always@(posedge w_clk) begin
        if(r_state==START_2) begin
            
        end
    end

/************************  EXECUTION  ************************/

/************************  ALU_0  ************************/
    m_ALU ALU(w_clk,r_rrs1,r_rrs2,w_ctl,);

    m_memory dmem(w_clk, w_addr, w_we, w_din, r_dout);


/************************  WRITEBACK  ************************/

    always@(posedge w_clk) begin
        if(r_state==WB_0) begin
        
        end
    end

endmodule


module m_memory (w_clk, w_addr, w_we, w_din, r_dout);
    input wire w_clk, w_we;
    input wire [10:0] w_addr;
    input wire [31:0] w_din;
    output reg [31:0] r_dout;
    reg [31:0] cm_ram [0:2047]; // 4K word (2048 x 32bit) memory
    always @(posedge w_clk) if (w_we) cm_ram[w_addr] <= w_din;
    always @(posedge w_clk) r_dout <= cm_ram[w_addr];
endmodule

module m_regfile (w_clk, w_rr1, w_rr2, w_wr, w_we, w_wdata, w_rdata1, w_rdata2);
    input wire w_clk;
    input wire [4:0] w_rr1, w_rr2, w_wr;
    input wire [31:0] w_wdata;
    input wire w_we;
    output wire [31:0] w_rdata1, w_rdata2;
    reg [31:0] r[0:31];
    assign w_rdata1 = (w_rr1==0) ? 0 : r[w_rr1];
    assign w_rdata2 = (w_rr2==0) ? 0 : r[w_rr2];
    always @(posedge w_clk) if(w_we) r[w_wr] <= w_wdata;
endmodule
