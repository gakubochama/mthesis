/********************************************************************************************/
`include "config.vh"
/********************************************************************************************/
`default_nettype none
/********************************************************************************************/

module UltraSmall(w_clk, w_rst_x, r_rout, r_halt, w_i_addr, w_d_addr, w_i_in, w_d_in, w_d_out, w_d_we);
    input wire w_clk,w_rst_x;
    output reg [31:0] r_rout;
    output reg r_halt;
    output wire [31:0] w_i_addr,w_d_addr;
    input wire [31:0] w_i_in,w_d_in;
    output wire [31:0] w_d_out;
    output wire [3:0] w_d_we;

    assign w_i_addr = r_pc;
    wire [31:0] w_ir = w_i_in;
    /*********************************** Register *******************************************/
    reg [31:0] r_pc = 0; //program counter
    reg [31:0] r_npc = 0; //program counter
    reg r_pc_we = 0;
    reg [4:0] r_state = 0;
    reg [5:0] r_cnt = 1; 
    reg [31:0] r_rrs1,r_rrs2; //value of rs1 rs2
    reg [9:0] r_alu_ctrl;
    reg [6:0] r_bru_ctrl;
    reg [31:0] r_imm;
    reg r_mem_we,r_reg_we,r_op_ld,r_op_imm;
    reg [31:0] r_shiftrega = 0;
    reg [31:0] r_shiftregb = 4;
    reg [2:0] r_carry = 0;
    reg [31:0] r_result = 0;
    reg r_bmis = 0;


    /****************************************************************************************/
    reg r_rst;
    always @(posedge w_clk) r_rst <= !w_rst_x | r_halt;

    /*********************************** START_1 ********************************************/

    /****************************************************************************************/
    wire [4:0] w_rd,w_rs1,w_rs2;
    wire w_mem_we,w_reg_we,w_op_ld,w_op_imm;
    wire [31:0] w_rrs1,w_rrs2;
    wire [9:0] w_alu_ctrl;
    wire [6:0] w_bru_ctrl;
    wire [31:0] w_imm;

    decoder decoder0(w_ir,w_rd,w_rs1,w_rs2,w_mem_we,w_reg_we,w_op_ld,w_op_imm,w_alu_ctrl,w_bru_ctrl,w_imm);

    regfile regfile0(w_clk,w_rs1,w_rs2,w_rrs1,w_rrs2,1'b0,w_rd,0);
    
    always @(posedge w_clk) begin 
        if(r_state==`START_1) begin
            r_rrs1 <= w_rrs1;
            r_rrs2 <= w_rrs2;
            r_mem_we <= w_mem_we;
            r_reg_we <= w_reg_we;
            r_op_ld <= w_op_ld;
            r_op_imm <= w_op_imm;
            r_alu_ctrl <= w_alu_ctrl;
            r_bru_ctrl <= w_bru_ctrl;
            r_imm <= w_imm;
            r_state <= `START_2;
        end
    end
    /*********************************** START_2 ********************************************/
    function [1:0] f_mux_34to2;
        input [31:0] w_in;
        input w_en;
        input [5:0] w_sel;
        
        begin
            if(w_en) begin
                case(w_sel)
                    1 : f_mux_34to2 = w_in[1:0];
                    2 : f_mux_34to2 = w_in[3:2];
                    3 : f_mux_34to2 = w_in[5:4];
                    4 : f_mux_34to2 = w_in[7:6];
                    5 : f_mux_34to2 = w_in[9:8];
                    6 : f_mux_34to2 = w_in[11:10];
                    7 : f_mux_34to2 = w_in[13:12];
                    8 : f_mux_34to2 = w_in[15:14];
                    9 : f_mux_34to2 = w_in[17:16];
                    10 : f_mux_34to2 = w_in[19:18];
                    11 : f_mux_34to2 = w_in[21:20];
                    12 : f_mux_34to2 = w_in[23:22];
                    13 : f_mux_34to2 = w_in[25:24];
                    14 : f_mux_34to2 = w_in[27:26];
                    15 : f_mux_34to2 = w_in[29:28];
                    16 : f_mux_34to2 = w_in[31:30];
                    default : f_mux_34to2 = 2'bxx;
                endcase
            end else begin
                f_mux_34to2 = 2'b00;
            end
        end
    endfunction

    wire [1:0] w_rrs1toshiftreg = f_mux_34to2(r_rrs1,(r_state==START_2),r_cnt);
    wire [1:0] w_rrs2toshiftreg = f_mux_34to2((r_op_imm) ? r_imm : r_rrs2,(r_state==START_2),r_cnt);

    function [1:0] f_mux_8to2;
        input [1:0] w_in1;
        input [1:0] w_in2;
        input [1:0] w_in3;
        input [1:0] w_in4;
        input [2:0] w_sel;

        begin
            case(w_sel)
                1 : f_mux_8to2 = w_in1;
                2 : f_mux_8to2 = w_in2;
                3 : f_mux_8to2 = w_in3;
                4 : f_mux_8to2 = w_in4;
                default : f_mux_8to2 = 2'bxx;
            endcase
        end
    endfunction

    function [1:0] f_mux_6to2;
        input [1:0] w_in1;
        input [1:0] w_in2;
        input [1:0] w_in3;
        input [1:0] w_sel;

        begin
            case(w_sel)
                1 : f_mux_8to2 = w_in1;
                2 : f_mux_8to2 = w_in2;
                3 : f_mux_8to2 = w_in3;
                default : f_mux_8to2 = 2'bxx;
            endcase
        end
    endfunction

    wire [1:0] w_in_shiftrega = f_mux_8to2(w_rrs1toshiftreg,w_alu_result,w_shiftrega_2,w_tmp_result,w_sel1);
    wire [1:0] w_in_shiftregb = f_mux_6to2(w_rrs2toshiftreg,w_imm_2,w_shiftregb_2,w_sel2);

    always@(posedge w_clk) begin
        if(r_state==`START_2) begin
            if(r_cnt[4]==0) begin
                {r_carry,r_npc} <= {r_pc[1:0] + r_shiftregb[1:0] + r_carry,r_pc[31:2]};
                r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                r_shiftregb <= {w_in_shiftregb,r_shiftregb[31:2]};
                r_cnt <= r_cnt + 1;
            end else begin
                {r_carry,r_npc} <= {r_npc[1:0] + r_shiftregb[1:0] + r_carry,r_npc[31:2]};
                r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                r_shiftregb <= {w_in_shiftregb,r_shiftregb[31:2]};
                r_cnt <= 1;
                r_carry <= 0;
                r_state <= `START_3;
            end
        end
    end



    /*********************************** START_3 ********************************************/
    always@(posedge w_clk) begin
        if(r_state==`START_3) begin
            if(r_alu_ctrl!=0 | r_alu_ctrl!=6 | r_alu_ctrl!=7) begin 
                r_state <= (r_op_imm) ? `ALUI_1 : `ALU_1;
            end
            else if(r_alu_ctrl==6) r_state <= `SHIFTL_1;
            else if(r_alu_ctrl==7) r_state <= `SHIFTR_1;
            else if(r_bru_ctrl!=0) r_state <= `BRANCH_1;
        end
    end

    /*********************************** EXECUTION ******************************************/
 
    /*********************************** ALU_1 **********************************************/
    wire [1:0] w_alu_result;
    wire w_carry;
    assign {w_carry,w_alu_result} = r_shiftrega[1:0] + r_shiftregb[1:0] + r_carry

    always @(posedge w_clk) begin
        if(r_state==`ALU_1) begin
            if(r_cnt[4]==0) begin 
                case(r_alu_ctrl)
                    1 : begin //add
                            {r_carry,r_shiftrega} <= {w_carry,w_in_shiftrega,r_shiftrega[31:2]};
                        end
                    2 : begin //sub
                            {r_carry,r_shiftrega} <= {r_shiftrega[1:0] - r_shiftregb[1:0] - r_carry,r_shiftrega[31:2]};
                        end
                    3 : begin //xor
                            r_shiftrega <= {r_shiftrega[1:0] ^ r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    4 : begin //or
                            r_shiftrega <= {r_shiftrega[1:0] | r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    5 : begin //and
                            r_shiftrega <= {r_shiftrega[1:0] & r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    default : begin 
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};
                        end
                endcase
                r_shiftregb <= {2'b00,r_shiftregb[31:2]};
                r_cnt <= r_cnt + 1;
            end else begin
                case(r_alu_ctrl)
                    1 : begin //add
                            {r_carry,r_shiftrega} <= {r_shiftrega[1:0] + r_shiftregb[1:0] + r_carry,r_shiftrega[31:2]};
                        end
                    2 : begin //sub
                            {r_carry,r_shiftrega} <= {r_shiftrega[1:0] - r_shiftregb[1:0] - r_carry,r_shiftrega[31:2]};
                        end
                    3 : begin //xor
                            r_shiftrega <= {r_shiftrega[1:0] ^ r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    4 : begin //or
                            r_shiftrega <= {r_shiftrega[1:0] | r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    5 : begin //and
                            r_shiftrega <= {r_shiftrega[1:0] & r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    default : begin 
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};
                        end
                endcase
                r_shiftregb <= {2'b00,r_shiftregb[31:2]};
                r_state <= `WRITEBACK_1;
                r_cnt <= 1;
            end
        end
    end    

    /*********************************** ALUI_1 *********************************************/
    always @(posedge w_clk) begin
        if(r_state==`ALUI_1) begin
            if(r_cnt[4]==0) begin 
                case(r_alu_ctrl)
                    1 : begin //add
                            {r_carry,r_shiftrega} <= {r_shiftrega[1:0] + r_shiftregb[1:0] + r_carry,r_shiftrega[31:2]};
                        end
                    2 : begin //sub
                            {r_carry,r_shiftrega} <= {r_shiftrega[1:0] - r_shiftregb[1:0] - r_carry,r_shiftrega[31:2]};
                        end
                    3 : begin //xor
                            r_shiftrega <= {r_shiftrega[1:0] ^ r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    4 : begin //or
                            r_shiftrega <= {r_shiftrega[1:0] | r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    5 : begin //and
                            r_shiftrega <= {r_shiftrega[1:0] & r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    default : begin 
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};
                        end
                endcase
                r_shiftregb <= {2'b00,r_shiftregb[31:2]};
                r_cnt <= r_cnt + 1;
            end else begin
                case(r_alu_ctrl)
                    1 : begin //add
                            {r_carry,r_shiftrega} <= {r_shiftrega[1:0] + r_shiftregb[1:0] + r_carry,r_shiftrega[31:2]};
                        end
                    2 : begin //sub
                            {r_carry,r_shiftrega} <= {r_shiftrega[1:0] - r_shiftregb[1:0] - r_carry,r_shiftrega[31:2]};
                        end
                    3 : begin //xor
                            r_shiftrega <= {r_shiftrega[1:0] ^ r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    4 : begin //or
                            r_shiftrega <= {r_shiftrega[1:0] | r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    5 : begin //and
                            r_shiftrega <= {r_shiftrega[1:0] & r_shiftregb[1:0],r_shiftrega[31:2]};
                        end
                    default : begin 
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};
                        end
                endcase
                r_shiftregb <= {2'b00,r_shiftregb[31:2]};
                r_state <= `WRITEBACK_1;
                r_cnt <= 1;
            end
        end
    end
    /*********************************** BRANCH_1 *******************************************/

    /*********************************** WRITEBACK_1 ****************************************/
    always @(posedge w_clk) begin
        if(r_state==`WRITEBACK_1) begin
            r_result <= r_shiftrega;
            r_shiftrega <= 0;
            r_state <= `START_1;
        end
    end

    /*********************************** OTHERS *********************************************/
    wire [6:0] w_op = w_ir[6:0];

    always @(posedge w_clk) begin
        if(r_rst) r_halt <= 0;
        else if (w_op==`OPCODE_HALT____) r_halt <= 1;
    end
    
    always @(posedge w_clk) begin
        if(r_rst) r_rout <= 0;
        else r_rout <= r_rout;
    end

endmodule


/********************************************************************************************/
module decoder(w_ir,w_rd,w_rs1,w_rs2,w_mem_we,w_reg_we,w_op_ld,w_op_imm,r_alu_ctrl,r_bru_ctrl,w_imm);
    input wire [31:0] w_ir;
    output wire [4:0] w_rd,w_rs1,w_rs2;
    output wire [31:0] w_imm;
    output wire w_mem_we,w_reg_we,w_op_ld,w_op_imm;
    output reg [9:0] r_alu_ctrl;
    output reg [6:0] r_bru_ctrl;

    //select format
    wire [4:0] w_op = w_ir[6:2]; // use 5-bit, because lower 2-bit are always 2'b11
    wire [2:0] w_funct3 = w_ir[14:12];
    wire [6:0] w_funct7 = w_ir[31:25];

    wire w_r_type = (w_op==5'b01100);
    wire w_s_type = (w_op[4:2]==3'b010); // (w_op==5'b01000);
    wire w_b_type = (w_op==5'b11000);
    wire w_j_type = (w_op==5'b11011);
    wire w_u_type = ({w_op[4], w_op[2:0]} ==4'b0101);
    wire w_i_type = (w_op==5'b11001 || w_op==5'b00000 || w_op==5'b00100);

    wire [31:0] w_imm_U = (w_u_type) ? {w_ir[31:12], 12'b0} : 0;
    wire [31:0] w_imm_I = (w_i_type) ? {{21{w_ir[31]}}, w_ir[30:20]} : 0;
    wire [31:0] w_imm_S = (w_s_type) ? {{21{w_ir[31]}}, w_ir[30:25], w_ir[11:7]} : 0;
    wire [31:0] w_imm_B = (w_b_type) ? {{20{w_ir[31]}}, w_ir[7], w_ir[30:25] ,w_ir[11:8], 1'b0} : 0;
    wire [31:0] w_imm_J = (w_j_type) ? {{12{w_ir[31]}}, w_ir[19:12], w_ir[20], w_ir[30:21], 1'b0} : 0;

    assign w_reg_we = (w_ir[11:7]!=0) & (w_op[3:0]!=4'b1000);  //!w_s_type && !w_b_type;
    assign w_mem_we = w_s_type;
    assign w_op_ld  = (w_op==5'b00000);
    assign w_op_imm = (w_op==5'b00100);
    assign w_rd     = (w_reg_we) ? w_ir[11:7] : 5'd0;
    assign w_rs1    = w_ir[19:15]; // (!w_u_type && !w_j_type)       ? w_ir[19:15] : 5'd0;
    assign w_rs2    = (!w_op_imm) ? w_ir[24:20] : 5'd0;
    assign w_imm = w_imm_U ^ w_imm_I ^ w_imm_S ^ w_imm_B ^ w_imm_J;

    reg [3:0] r_alu_ctrl0;
    always @(*) begin
        case(w_op)
            5'b01100 : r_alu_ctrl0 = {w_funct7[5], w_funct3}; 
            5'b00100 : r_alu_ctrl0 = (w_funct3==3'h5) ? {w_funct7[5], w_funct3} : {1'b0, w_funct3};
            default  : r_alu_ctrl0 = 4'b1111;
        endcase
    end

    always @(*) begin
        case(r_alu_ctrl0)
            `ALU_CTRL_ADD___ : r_alu_ctrl = 1;
            `ALU_CTRL_SUB___ : r_alu_ctrl = 2;
            `ALU_CTRL_XOR___ : r_alu_ctrl = 3;
            `ALU_CTRL_OR____ : r_alu_ctrl = 4;
            `ALU_CTRL_AND___ : r_alu_ctrl = 5;
            `ALU_CTRL_SLL___ : r_alu_ctrl = 6;
            `ALU_CTRL_SRL___ : r_alu_ctrl = 7;
            default          : r_alu_ctrl = 0;
        endcase
    end
    
    always @(*) begin /***** one-hot encoding *****/
        case(w_op)
            5'b11011 : r_bru_ctrl =                    7'b1000000;     // JAL  -> taken
            5'b11001 : r_bru_ctrl =                    7'b1000000;     // JALR -> taken
            5'b11000 : r_bru_ctrl = (w_funct3==3'b000) ? 7'b0000001 :    // BEQ
                                  (w_funct3==3'b001) ? 7'b0000010 :    // BNE
                                  (w_funct3==3'b100) ? 7'b0000100 :    // BLT
                                  (w_funct3==3'b101) ? 7'b0001000 :    // BGE
                                  (w_funct3==3'b110) ? 7'b0010000 :    // BLTU
                                  (w_funct3==3'b111) ? 7'b0100000 : 0; // BGEU
            default : r_bru_ctrl = 0;
        endcase
    end
endmodule

/********************************************************************************************/  
module regfile(w_clk, w_rs1, w_rs2, w_rdata1, w_rdata2, w_we, w_rd, w_wdata);
    input  wire        w_clk;
    input  wire [ 4:0] w_rs1, w_rs2;
    output wire [31:0] w_rdata1, w_rdata2;
    input  wire        w_we;
    input  wire [ 4:0] w_rd;
    input  wire [31:0] w_wdata;

    reg [31:0] mem [0:31];
    assign w_rdata1 = (w_rs1 == 0) ? 0 : (w_rs1==w_rd) ? w_wdata : mem[w_rs1];
    assign w_rdata2 = (w_rs2 == 0) ? 0 : (w_rs2==w_rd) ? w_wdata : mem[w_rs2];
    always @(posedge w_clk) if(w_we && (w_rd!=0)) mem[w_rd] <= w_wdata;
endmodule