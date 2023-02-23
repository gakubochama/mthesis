///********************************************************************************************/
//`include "config.vh"
///********************************************************************************************/
//`timescale 1ns/100ps
//`default_nettype none
///********************************************************************************************/
//
//module m_verilator_testbench(w_clk,w_rst);
//
//    reg r_core_rst;
//    always @(posedge r_clk) r_core_rst <= (w_rst | 1'b0);
//
//    wire w_i_en;
//    wire [31:0] w_rout, w_i_addr, w_d_addr, w_i_data, w_d_data, w_wd_data;
//    wire [3:0] w_d_we;
//
//    UltraSmall p(r_clk, !r_core_rst, w_rout, w_i_addr, w_d_addr, w_i_data, w_d_data, w_wd_data, w_i_en, w_d_we);
//
//    wire [31:0] tmpdata;
//
//    m_IMEM#(32,`MEM_SIZE/4) imem(r_clk, 1'b0 , w_i_en, 9'd0, w_i_addr[$clog2(`MEM_SIZE)-1:2], 32'd0, w_i_data);
//
//    m_DMEM#(32,`MEM_SIZE/4) dmem(r_clk, 1'b0 , 4'd0, 9'd0, 32'd0, tmpdata, 
//                                r_clk, !r_core_rst, w_d_we, w_d_addr[$clog2(`MEM_SIZE)-1:2], w_wd_data, w_d_data);
//endmodule
//
///********************************************************************************************/
//module m_IMEM#(parameter WIDTH=32, ENTRY=256)(CLK, WE, EN, WADDR, RADDR, IDATA, ODATA);
//    input  wire                     CLK, WE ,EN;
//    input  wire [$clog2(ENTRY)-1:0] WADDR;
//    input  wire [$clog2(ENTRY)-1:0] RADDR;
//    input  wire [WIDTH-1:0]         IDATA;
//    output wire [WIDTH-1:0]         ODATA;
//    
//`include "constants.v"
//
//    reg [WIDTH-1:0] mem[0:ENTRY-1];
//    reg [$clog2(ENTRY)-1:0] r_raddr = START_PC_ADDR/4;
//    
//    always @(posedge CLK) begin
//        if (WE) mem[WADDR] <= IDATA;
//        if (EN) r_raddr <= RADDR;
//        //ODATA <= mem[RADDR];
//    end
//    assign ODATA = mem[r_raddr];
//endmodule
//
///********************************************************************************************/
//module m_DMEM#(parameter WIDTH=32, ENTRY=256)(CLK1, EN1, WE1, ADDR1, IDATA1, ODATA1, 
//                                              CLK2, EN2, WE2, ADDR2, IDATA2, ODATA2);
//    input  wire                     CLK1, EN1;
//    input  wire [3:0]               WE1;
//    input  wire [$clog2(ENTRY)-1:0] ADDR1;
//    input  wire [WIDTH-1:0]         IDATA1;
//    output reg  [WIDTH-1:0]         ODATA1;
//    input  wire                     CLK2, EN2;
//    input  wire [3:0]               WE2;
//    input  wire [$clog2(ENTRY)-1:0] ADDR2;
//    input  wire [WIDTH-1:0]         IDATA2;
//    output wire  [WIDTH-1:0]         ODATA2;
//    
//     reg [WIDTH-1:0] mem[0:ENTRY-1];
//    always @(posedge CLK1) begin
//        if(EN1) begin
//            if (WE1[0]) mem[ADDR1][ 7: 0] <= IDATA1[ 7: 0];
//            if (WE1[1]) mem[ADDR1][15: 8] <= IDATA1[15: 8];
//            if (WE1[2]) mem[ADDR1][23:16] <= IDATA1[23:16];
//            if (WE1[3]) mem[ADDR1][31:24] <= IDATA1[31:24];
//            ODATA1 <= mem[ADDR1];
//        end
//    end
//
//    always @(posedge CLK2) begin
//        if(EN2) begin
//            if (WE2[0]) mem[ADDR2][ 7: 0] <= IDATA2[ 7: 0];
//            if (WE2[1]) mem[ADDR2][15: 8] <= IDATA2[15: 8];
//            if (WE2[2]) mem[ADDR2][23:16] <= IDATA2[23:16];
//            if (WE2[3]) mem[ADDR2][31:24] <= IDATA2[31:24];
//            //ODATA2 <= mem[ADDR2];
//        end
//    end
//    assign ODATA2 = mem[ADDR2];
//endmodule