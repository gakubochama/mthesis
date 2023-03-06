/********************************************************************************************/
`include "config.vh"
/********************************************************************************************/
`timescale 1ns/100ps
`default_nettype none
/********************************************************************************************/
`ifndef TRACE
module m_iverilog_testbench();
    reg r_clk=0; initial forever #50 r_clk = ~r_clk;
    reg r_rst=0;

/*
    always begin
        #50 r_clk = ~r_clk;
    end
*/

    reg r_core_rst;
    always @(posedge r_clk) r_core_rst <= (r_rst | 1'b0);

    wire w_i_en;
    wire [31:0] w_rout, w_i_addr, w_d_addr, w_i_data, w_d_data, w_wd_data;
    wire [3:0] w_d_we;

    initial begin
        $dumpfile("main.vcd");
        $dumpvars(0,m_iverilog_testbench);
    //initialize imem

        //imem.mem[0] = 32'h01ee8fb3; //add x31,x29,x30
        //imem.mem[0] = 32'h41ee8fb3; //sub x31, x29, x30
        //imem.mem[0] = 32'h01eecfb3; //xor x31, x29, x30
        //imem.mem[0] = 32'h01eeefb3; //or x31, x29, x30
        //imem.mem[0] = 32'h01eeffb3; //and x31, x29, x30

        //imem.mem[0] = 32'hfffe8f93; //addi x31,x29,-1
        //imem.mem[0] = 32'h00fe8f93; //addi x31, x29, 15
        //imem.mem[0] = 32'h00fecf93; //xori x31, x29, 15
        //imem.mem[0] = 32'h00feef93; //ori x31, x29, 15
        //imem.mem[0] = 32'h00feff93; //andi x31, x29, 15
        //imem.mem[0] = 32'h00002fb7; //lui x31,2
        //imem.mem[1] = 32'h00002f97; //auipc x31,2

        //set less than
        //imem.mem[0] = 32'h01eeafb3; //slt x31,x29,x30
        //imem.mem[0] = 32'h01eebfb3; //sltu x31,x29,x30
        //imem.mem[0] = 32'h003eaf93; //slti x31,x29,3
        //imem.mem[0] = 32'h003ebf93; //sltiu x31,x29,3

        //shift
        //imem.mem[0] = 32'h01cedfb3; //srl x31, x29 ,x28
        //imem.mem[0] = 32'h41cedfb3; //sra x31,x29,x28  
        //imem.mem[0] = 32'h003edf93; //srli x31,x29,3
        //imem.mem[0] = 32'h403edf93; //srai x31,x29,3
        //imem.mem[0] = 32'h01ce9fb3; //sll x31,x29,x28
        //imem.mem[0] = 32'h003e9f93; //slli x31,x29,3

        //jump
        //imem.mem[2] = 32'hff9ff0ef; //jal x1,-8
        //imem.mem[2] = 32'hff8e80e7; //jalr x1,x29,-8

        //branch
        //imem.mem[2] = 32'hffee8ce3; //beq x29,x30,-8
        //imem.mem[2] = 32'hffee9ce3; //bne x29,x30,-8
        //imem.mem[2] = 32'hffeecce3; //blt x29,x30,-8
        //imem.mem[2] = 32'hffeedce3; //bge x29,x30,-8
        //imem.mem[2] = 32'hffeeece3; //bltu x29,x30,-8
        //imem.mem[2] = 32'hffeefce3; //bgeu x29,x30,-8

        //imem.mem[2] = 32'hffd01ce3; //bne x0,x29,1
        //imem.mem[3] = 32'hffe01ae3; //bne x0,x30,1

        //store
        //imem.mem[0] = 32'h01e18223; //sb x30,4(x3)
        //imem.mem[0] = 32'h01e19223; //sh x30,4(x3)
        //imem.mem[0] = 32'h01e1a223; //sw x30,4(x3)

        //load
        //imem.mem[0] = 32'h00418f83; //lb x31,4(x3)
        //imem.mem[0] = 32'h00518f83; //lb x31,5(x3)
        //imem.mem[0] = 32'h00618f83; //lb x31,6(x3)
        //imem.mem[0] = 32'h00718f83; //lb x31,7(x3)
        //imem.mem[0] = 32'h00419f83; //lh x31,4(x3)
        //imem.mem[0] = 32'h00619f83; //lh x31,6(x3)
        //imem.mem[0] = 32'h0041af83; //lw x31,4(x3)
        imem.mem[0] = 32'h0041cf83; //lbu x31,4(x3)
        //imem.mem[0] = 32'h0051cf83; //lbu x31,5(x3)
        //imem.mem[0] = 32'h0061cf83; //lbu x31,6(x3)
        //imem.mem[0] = 32'h0071cf83; //lbu x31,7(x3)
        //imem.mem[0] = 32'h0041df83; //lhu x31,4(x3)
        //imem.mem[0] = 32'h0061df83; //lhu x31,6(x3)


    //initialize dmem
        dmem.mem[0] = 32'h00000000;
        dmem.mem[1] = 32'h72726f63;
        dmem.mem[2] = 32'h00000000;
        dmem.mem[3] = 32'h00000000;
        dmem.mem[4] = 32'h00000000;

    //initialize regfile
        p.regfile0.mem[0] = 32'h00000000; //$zero
        p.regfile0.mem[1] = 32'h00000000; //$ra
        p.regfile0.mem[2] = 32'h7ffffff0; //$sp
        p.regfile0.mem[3] = 32'h10000000; //$gp
        p.regfile0.mem[4] = 32'h00000000; //$tp
        p.regfile0.mem[28] = 32'h00000000; 
        p.regfile0.mem[29] = 32'h00000000; 
        p.regfile0.mem[30] = 32'h00000001;
        p.regfile0.mem[31] = 32'h00000000; 
    end

/*
    initial $write("r_pc:%b\n",p.r_pc);

    initial $write("w_i_addr:%b\n",p.w_i_addr);

    initial #1 $write("w_i_in:%b\n",p.w_ir);

    initial #1 $write("w_rs1:%b\nw_rs2:%b\nw_rd:%b\n",p.w_rs1,p.w_rs2,p.w_rd);

    initial #1 $write("w_alu_ctrl:%b\n",p.w_alu_ctrl);

    initial #1 $write("w_rrs1:%b\nw_rrs2:%b\n",p.w_rrs1,p.w_rrs2);

    initial #51 $write("r_rrs1:%b\nr_rrs2:%b\n",p.r_rrs1,p.r_rrs2);
*/

    always@(posedge r_clk) begin
        //$write("%4d cycle r_state:%b\n",($time-50)/100,p.r_state);
        //#1 $write("%4d r_state:%b r_cnt:%b r_tmp:%b\n",$time,p.r_state,p.r_cnt,p.r_tmp);
        case(p.r_state)
            `START_1 : $write("%4d cycle state:START_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `START_2 : $write("%4d cycle state:START_2 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `START_3 : $write("%4d cycle state:START_3 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `ALU_1 : $write("%4d cycle state:ALU_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `ALUI_1 : $write("%4d cycle state:ALUI_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `BRANCH_1 : $write("%4d cycle state:BRANCH_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `JUMP_1 : $write("%4d cycle state:JUMP_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `SHIFTL_1 : $write("%4d cycle state:SHIFTL_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `SHIFTL_2 : $write("%4d cycle state:SHIFTL_2 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `SHIFTR_1 : $write("%4d cycle state:SHIFTR_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `SHIFTR_2 : $write("%4d cycle state:SHIFTR_2 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `SHIFTR_3 : $write("%4d cycle state:SHIFTR_3 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `LOADSTORE_1 : $write("%4d cycle state:LOADSTORE_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `LOADSTORE_2 : $write("%4d cycle state:LOADSTORE_2 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `LOAD_1 : $write("%4d cycle state:LOAD_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `LOAD_2 : $write("%4d cycle state:LOAD_2 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `STORE_1 : $write("%4d cycle state:STORE_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `STORE_2 : $write("%4d cycle state:STORE_2 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            `WRITEBACK_1 : $write("%4d cycle state:WRITEBACK_1 counter:%b shift_tmp:%b\n",($time-50)/100,p.r_cnt,p.r_tmp);
            default : $write("%4d cycle state:%b counter:%b shift_tmp:%b\n",($time-50)/100,p.r_state,p.r_cnt,p.r_tmp);
        endcase

        $write("pc: %b\n",p.r_pc);
        $write("npc:%b\n",p.r_npc);
        $write("imm:%b\n",p.w_imm);
        $write("ir: %b\n",p.w_ir);
        $write("rrs1:%b\nrrs2:%b\n",p.w_rrs1,p.w_rrs2);

//for jump instruction
        //$write("op_jal:%b\n",p.w_op_jal);
        //$write("op_jalr:%b\n",p.w_op_jalr);

//for load instruction
        //$write("op_lb:%b\n",p.w_op_lb);
        //$write("dmem_offset:%b\n",p.r_dmem_offset);
        //$write("shiftregb_msb:%b\n",p.r_shiftregb_msb);

//for branch instruction
        //$write("eq: %b\n",p.w_eq);
        //$write("ne: %b\n",p.w_ne);
        //$write("lt: %b\n",p.w_lt);
        //$write("ge: %b\n",p.w_ge);
        //$write("ltu: %b\n",p.w_ltu);
        //$write("geu: %b\n",p.w_geu);
        $write("w_op_bne: %b\n",p.w_op_bne);
        $write("branch_delay: %b\n",p.r_branch_delay);

//for slt instruction
        //$write("sge: %b\n",p.r_sge);
        //$write("sgeu: %b\n",p.r_sgeu);
        //$write("slt: %b\n",p.w_slt);
        //$write("sltu: %b\n",p.w_sltu);

//for auipc
        //$write("auipc_delay:%b\n",p.r_auipc_delay);

//for shift instruction
        //$write("shift_ctrl:%b\n",p.w_shift_ctrl);
        //$write("reg_we:%b\n",p.w_reg_we);
        //$write("wdata:%b\n",p.w_wdata);

//for update pc
        //$write("pcupate: %b\n",p.r_pcupdate);



/* memo
        $write("imm:%b\n",p.w_imm);
        $write("d_we:%b\n",p.w_d_we);
        $write("d_addr:%b\n",p.w_d_addr);
        $write("d_in:%b\n",p.w_d_in);
        $write("d_out:%b\n",p.w_d_out);
        $write("reg_we:%b\n",p.w_reg_we);
        $write("mem_we:%b\n",p.w_mem_we);
        $write("i_en:%b\n",p.w_i_en);
        $write("alu_ctrl:%b\n",p.w_alu_ctrl);
        $write("shift_ctrl:%b\n",p.w_shift_ctrl);
        $write("bru_ctrl:%b\n",p.w_bru_ctrl);
        $write("reg_we:%b\n",p.w_reg_we);
        $write("mem_we:%b\n",p.w_mem_we);
        $write("op_ld:%b\n",p.w_op_ld);
        $write("op_imm:%b\n",p.w_op_imm);
        //$write("shiftregA:%b\nshiftregB:%b\n",p.r_shiftrega,p.r_shiftregb);
        //$write("rd:%b\n",p.w_rd);
        //$write("result:%b\n",p.w_result);
        //$write("rout:%b\n",p.w_rout);
        //$write("carry:%b\n",p.r_carry);
*/
    end


/*
    initial #1 $write("dmem[0]:%h\n",dmem.mem[0]);
    initial #1 $write("dmem[1]:%h\n",dmem.mem[1]);
    initial #1 $write("dmem[2]:%h\n",dmem.mem[2]);
    initial #1 $write("dmem[3]:%h\n",dmem.mem[3]);
    initial #1 $write("dmem[4]:%h\n",dmem.mem[4]);
    initial #1 $write("regfile[0]:%h\n",p.regfile0.mem[0]);
    initial #1 $write("regfile[1]:%h\n",p.regfile0.mem[1]);
    initial #1 $write("regfile[2]:%h\n",p.regfile0.mem[2]);
    initial #1 $write("regfile[3]:%h\n",p.regfile0.mem[3]);
    initial #1 $write("regfile[4]:%h\n",p.regfile0.mem[4]);
    initial #1 $write("regfile[5]:%h\n",p.regfile0.mem[5]);
    initial #1 $write("regfile[6]:%h\n",p.regfile0.mem[6]);
    initial #1 $write("regfile[7]:%h\n",p.regfile0.mem[7]);
    initial #1 $write("regfile[8]:%h\n",p.regfile0.mem[8]);
    initial #1 $write("regfile[9]:%h\n",p.regfile0.mem[9]);
    initial #1 $write("regfile[10]:%h\n",p.regfile0.mem[10]);
    initial #1 $write("regfile[11]:%h\n",p.regfile0.mem[11]);
    initial #1 $write("regfile[12]:%h\n",p.regfile0.mem[12]);
    initial #1 $write("regfile[13]:%h\n",p.regfile0.mem[13]);
    initial #1 $write("regfile[14]:%h\n",p.regfile0.mem[14]);
    initial #1 $write("regfile[15]:%h\n",p.regfile0.mem[15]);
    initial #1 $write("regfile[16]:%h\n",p.regfile0.mem[16]);
    initial #1 $write("regfile[17]:%h\n",p.regfile0.mem[17]);
    initial #1 $write("regfile[18]:%h\n",p.regfile0.mem[18]);
    initial #1 $write("regfile[19]:%h\n",p.regfile0.mem[19]);
    initial #1 $write("regfile[20]:%h\n",p.regfile0.mem[20]);
    initial #1 $write("regfile[21]:%h\n",p.regfile0.mem[21]);
    initial #1 $write("regfile[22]:%h\n",p.regfile0.mem[22]);
    initial #1 $write("regfile[23]:%h\n",p.regfile0.mem[23]);
    initial #1 $write("regfile[24]:%h\n",p.regfile0.mem[24]);
    initial #1 $write("regfile[25]:%h\n",p.regfile0.mem[25]);
    initial #1 $write("regfile[26]:%h\n",p.regfile0.mem[26]);
    initial #1 $write("regfile[27]:%h\n",p.regfile0.mem[27]);
    initial #1 $write("regfile[28]:%h\n",p.regfile0.mem[28]);
    initial #1 $write("regfile[29]:%h\n",p.regfile0.mem[29]);
    initial #1 $write("regfile[30]:%h\n",p.regfile0.mem[30]);
    initial #1 $write("regfile[31]:%h\n",p.regfile0.mem[31]);
*/

    initial #15000 $write("dmem[0]:%h\n",dmem.mem[0]);
    initial #15000 $write("dmem[1]:%h\n",dmem.mem[1]);
    initial #15000 $write("dmem[2]:%h\n",dmem.mem[2]);
    initial #15000 $write("dmem[3]:%h\n",dmem.mem[3]);
    initial #15000 $write("dmem[4]:%h\n",dmem.mem[4]);
    initial #15000 $write("regfile[0]:%h\n",p.regfile0.mem[0]);
    initial #15000 $write("regfile[1]:%h\n",p.regfile0.mem[1]);
    initial #15000 $write("regfile[2]:%h\n",p.regfile0.mem[2]);
    initial #15000 $write("regfile[3]:%h\n",p.regfile0.mem[3]);
    initial #15000 $write("regfile[4]:%h\n",p.regfile0.mem[4]);
    initial #15000 $write("regfile[5]:%h\n",p.regfile0.mem[5]);
    initial #15000 $write("regfile[6]:%h\n",p.regfile0.mem[6]);
    initial #15000 $write("regfile[7]:%h\n",p.regfile0.mem[7]);
    initial #15000 $write("regfile[8]:%h\n",p.regfile0.mem[8]);
    initial #15000 $write("regfile[9]:%h\n",p.regfile0.mem[9]);
    initial #15000 $write("regfile[10]:%h\n",p.regfile0.mem[10]);
    initial #15000 $write("regfile[11]:%h\n",p.regfile0.mem[11]);
    initial #15000 $write("regfile[12]:%h\n",p.regfile0.mem[12]);
    initial #15000 $write("regfile[13]:%h\n",p.regfile0.mem[13]);
    initial #15000 $write("regfile[14]:%h\n",p.regfile0.mem[14]);
    initial #15000 $write("regfile[15]:%h\n",p.regfile0.mem[15]);
    initial #15000 $write("regfile[16]:%h\n",p.regfile0.mem[16]);
    initial #15000 $write("regfile[17]:%h\n",p.regfile0.mem[17]);
    initial #15000 $write("regfile[18]:%h\n",p.regfile0.mem[18]);
    initial #15000 $write("regfile[19]:%h\n",p.regfile0.mem[19]);
    initial #15000 $write("regfile[20]:%h\n",p.regfile0.mem[20]);
    initial #15000 $write("regfile[21]:%h\n",p.regfile0.mem[21]);
    initial #15000 $write("regfile[22]:%h\n",p.regfile0.mem[22]);
    initial #15000 $write("regfile[23]:%h\n",p.regfile0.mem[23]);
    initial #15000 $write("regfile[24]:%h\n",p.regfile0.mem[24]);
    initial #15000 $write("regfile[25]:%h\n",p.regfile0.mem[25]);
    initial #15000 $write("regfile[26]:%h\n",p.regfile0.mem[26]);
    initial #15000 $write("regfile[27]:%h\n",p.regfile0.mem[27]);
    initial #15000 $write("regfile[28]:%h\n",p.regfile0.mem[28]);
    initial #15000 $write("regfile[29]:%h\n",p.regfile0.mem[29]);
    initial #15000 $write("regfile[30]:%h\n",p.regfile0.mem[30]);
    initial #15000 $write("regfile[31]:%h\n",p.regfile0.mem[31]);


    initial #15000 $finish;


    UltraSmall p(r_clk, !r_core_rst, w_rout, w_i_addr, w_d_addr, w_i_data, w_d_data, w_wd_data, w_i_en, w_d_we);

    wire [31:0] tmpdata;

    m_IMEM#(32,`MEM_SIZE/4) imem(r_clk, 1'b0 , w_i_en, 9'd0, w_i_addr[$clog2(`MEM_SIZE)-1:2], 32'd0, w_i_data);

    m_DMEM#(32,`MEM_SIZE/4) dmem(r_clk, 1'b0 , 4'd0, 9'd0, 32'd0, tmpdata, 
                                r_clk, !r_core_rst, w_d_we, w_d_addr[$clog2(`MEM_SIZE)-1:2], w_wd_data, w_d_data);
endmodule

/********************************************************************************************/
module m_IMEM#(parameter WIDTH=32, ENTRY=256)(CLK, WE, EN, WADDR, RADDR, IDATA, ODATA);
    input  wire                     CLK, WE ,EN;
    input  wire [$clog2(ENTRY)-1:0] WADDR;
    input  wire [$clog2(ENTRY)-1:0] RADDR;
    input  wire [WIDTH-1:0]         IDATA;
    output wire [WIDTH-1:0]         ODATA;
    
`include "constants.v"

    reg [WIDTH-1:0] mem[0:ENTRY-1];
    reg [$clog2(ENTRY)-1:0] r_raddr = START_PC_ADDR/4;
    
    always @(posedge CLK) begin
        if (WE) mem[WADDR] <= IDATA;
        if (EN) r_raddr <= RADDR;
        //ODATA <= mem[RADDR];
    end
    assign ODATA = mem[r_raddr];
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
    output wire  [WIDTH-1:0]         ODATA2;
    
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
            //ODATA2 <= mem[ADDR2];
        end
    end
    assign ODATA2 = mem[ADDR2];
endmodule
`endif