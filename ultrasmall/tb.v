/********************************************************************************************/
`include "config.vh"
/********************************************************************************************/
`timescale 1ns/100ps
`default_nettype none
/********************************************************************************************/

module m_testbench();
    reg r_clk=0; initial forever #50 r_clk = ~r_clk;
    reg r_rst=0;

/*
    always begin
        #50 r_clk = ~r_clk;
    end
*/

    reg r_core_rst;
    always @(posedge r_clk) r_core_rst <= (r_rst | 1'b0);

    wire w_halt;
    wire [31:0] w_rout, w_i_addr, w_d_addr, w_i_data, w_d_data, w_wd_data;
    wire [3:0] w_d_we;

    initial begin
    //initialize imem
        //imem.mem[0] = {7'b0000000,5'd2,5'd1,3'b000,5'd3,5'b01100,2'b11}; // add $3, $1, $2
        //imem.mem[1] = {7'b0100000,5'd2,5'd1,3'b000,5'd3,5'b01100,2'b11}; // sub $3, $1, $2
        //imem.mem[2] = {7'b0000000,5'd2,5'd1,3'b100,5'd3,5'b01100,2'b11}; // xor $3, $1, $2
        //imem.mem[3] = {7'b0000000,5'd2,5'd1,3'b110,5'd3,5'b01100,2'b11}; // or $3, $1, $2
        //imem.mem[4] = {7'b0000000,5'd2,5'd1,3'b111,5'd3,5'b01100,2'b11}; // and $3, $1, $2
        //imem.mem[5] = {7'b1111111,5'd5,5'd4,3'b000,4'b1100,1'b1,5'b11000,2'b11}; // beq $4, $5, -20
        imem.mem[0] = {12'd13,5'd1,3'b000,5'd3,5'b00100,2'b11}; // addi $3, $1,15

    //initialize dmem
        //dmem.mem[0] = {};

    //initialize regfile
        p.regfile0.mem[1] = {32'd13}; //1101
        p.regfile0.mem[2] = {32'd15}; //1111
        p.regfile0.mem[4] = {32'd0}; 
        p.regfile0.mem[5] = {32'd0}; 
    end

/*  
    initial begin
    #50000 $finish;
    end
*/

/*
    always@(posedge r_clk) begin
        $write("%4d cycle\nshitregA:%b\nshitregB:%b\n",($time-50)/100 + 1,p.r_shiftrega,p.r_shiftregb);
    end
*/

    //initial #8000 $write("%4d cycle\nrrs:%b\nrrt:%b\nresult:%b\n",($time-50)/100,p.r_rrs1,p.r_rrs2,p.r_result);

/*
    initial $write("r_pc:%b\n",p.r_pc);

    initial $write("w_i_addr:%b\n",p.w_i_addr);

    initial #1 $write("w_i_in:%b\n",p.w_ir);

    initial #1 $write("w_rs1:%b\nw_rs2:%b\nw_rd:%b\n",p.w_rs1,p.w_rs2,p.w_rd);

    initial #1 $write("w_alu_ctrl:%b\n",p.w_alu_ctrl);

    initial #1 $write("w_rrs1:%b\nw_rrs2:%b\n",p.w_rrs1,p.w_rrs2);

    initial #51 $write("r_rrs1:%b\nr_rrs2:%b\n",p.r_rrs1,p.r_rrs2);
*/

    //initial #6551 $write("r_state:%b\n",p.r_state);
    //initial #6751 $write("w_ir:%b\n",p.w_ir);
    //initial #1 $write("w_npc:%b\n",p.w_npc);
    //initial #6751 $write("w_alu_ctrl:%b\n",p.w_alu_ctrl);
    //initial #6751 $write("w_rrs1:%b\nw_rrs2:%b\n",p.w_rrs1,p.w_rrs2);

/*
    initial #51 $write("r_pc:%b\n",p.r_pc);
    initial #151 $write("r_pc:%b\n",p.r_pc);
    initial #6750 $write("r_result:%b\n",p.r_result);
    initial #6751 $write("r_pc:%b\n",p.r_pc);
    initial #6851 $write("r_pc:%b\n",p.r_pc);
    initial #13450 $write("r_result:%b\n",p.r_result);
    initial #20150 $write("r_result:%b\n",p.r_result);
    initial #26850 $write("r_result:%b\n",p.r_result);
    initial #33550 $write("r_result:%b\n",p.r_result);
    initial #33550 $write("w_ir:%b\n",p.w_ir);
    initial #33551 $write("r_bru_ctrl:%b\n",p.r_bru_ctrl);
    initial #33551 $write("r_imm:%b\n",p.r_imm);
    initial #33551 $write("r_pc:%b\n",p.r_pc);
    initial #33651 $write("r_pc:%b\n",p.r_pc);
    initial #40251 $write("r_pc:%b\n",p.r_pc);
    initial #40351 $write("r_pc:%b\n",p.r_pc);
*/

    always@(posedge r_clk) begin
        //$write("%4d cycle r_state:%b\n",($time-50)/100,p.r_state);
        $write("%4d r_state:%b\n",$time,p.r_state);
        $write("shitregA:%b\nshitregB:%b\n",p.r_shiftrega,p.r_shiftregb);
        //$write("r_cnt:%b\n",p.r_cnt);
    end

    //initial #1351 $write("r_rrs1:%b\nr_rrs2:%b\n",p.r_rrs1,p.r_rrs2);


    //initial #3051 $write("r_state:%b\n",p.r_state);
    //initial #6751 $write("w_ir:%b\n",p.w_ir);
    //initial #6551 $write("r_pc:%b\n",p.r_pc);
    initial #7000 $finish;

    UltraSmall p(r_clk, !r_core_rst, w_rout, w_halt, w_i_addr, w_d_addr, w_i_data, w_d_data, w_wd_data, w_d_we);

    wire [31:0] tmpdata;

    m_IMEM#(32,`MEM_SIZE/4) imem(r_clk, 1'b0 , 6'd0, w_i_addr[$clog2(`MEM_SIZE)-1:2], 32'd0, w_i_data);

    m_DMEM#(32,`MEM_SIZE/4) dmem(r_clk, 1'b0 , 4'd0, 6'd0, 32'd0, tmpdata, 
                                r_clk, !r_core_rst, w_d_we, w_d_addr[$clog2(`MEM_SIZE)-1:2], w_wd_data, w_d_data);



endmodule

/********************************************************************************************/
module m_IMEM#(parameter WIDTH=32, ENTRY=256)(CLK, WE, WADDR, RADDR, IDATA, ODATA);
    input  wire                     CLK, WE;
    input  wire [$clog2(ENTRY)-1:0] WADDR;
    input  wire [$clog2(ENTRY)-1:0] RADDR;
    input  wire [WIDTH-1:0]         IDATA;
    output wire [WIDTH-1:0]         ODATA;
    
    reg [WIDTH-1:0] mem[0:ENTRY-1];
    always @(posedge CLK) begin
        if (WE) mem[WADDR] <= IDATA;
        //ODATA <= mem[RADDR];
    end
    assign ODATA = mem[RADDR];
endmodule

/********************************************************************************************/
module m_DMEM#(parameter WIDTH=32, ENTRY=256)(CLK1, EN1, WE1, ADDR1, IDATA1, ODATA1, 
                                              CLK2, EN2, WE2, ADDR2, IDATA2, ODATA2);
    input  wire                     CLK1, EN1;
    input  wire [3:0]               WE1;
    input  wire [$clog2(ENTRY)-1:0] ADDR1;
    input  wire [WIDTH-1:0]         IDATA1;
    output reg  [WIDTH-1:0]         ODATA1;
    input  wire                     CLK2, EN2;
    input  wire [3:0]               WE2;
    input  wire [$clog2(ENTRY)-1:0] ADDR2;
    input  wire [WIDTH-1:0]         IDATA2;
    output reg  [WIDTH-1:0]         ODATA2;
    
     reg [WIDTH-1:0] mem[0:ENTRY-1];
    always @(posedge CLK1) begin
        if(EN1) begin
            if (WE1[0]) mem[ADDR1][ 7: 0] <= IDATA1[ 7: 0];
            if (WE1[1]) mem[ADDR1][15: 8] <= IDATA1[15: 8];
            if (WE1[2]) mem[ADDR1][23:16] <= IDATA1[23:16];
            if (WE1[3]) mem[ADDR1][31:24] <= IDATA1[31:24];
            ODATA1 <= mem[ADDR1];
        end
    end

    always @(posedge CLK2) begin
        if(EN2) begin
            if (WE2[0]) mem[ADDR2][ 7: 0] <= IDATA2[ 7: 0];
            if (WE2[1]) mem[ADDR2][15: 8] <= IDATA2[15: 8];
            if (WE2[2]) mem[ADDR2][23:16] <= IDATA2[23:16];
            if (WE2[3]) mem[ADDR2][31:24] <= IDATA2[31:24];
            ODATA2 <= mem[ADDR2];
        end
    end
endmodule