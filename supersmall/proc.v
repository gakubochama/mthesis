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

    /*********************************** Register *******************************************/
    reg [17:0] r_pc = 0; //program counter
    reg r_pc_we = 0;
    reg [4:0] r_state = 0;
    reg [5:0] r_cnt = 1; 
    reg [31:0] r_rrs1,r_rrs2; //value of rs1 rs2
    reg [9:0] r_alu_ctrl;
    reg [6:0] r_bru_ctrl;
    reg [31:0] r_imm;
    reg r_mem_we,r_reg_we,r_op_ld,r_op_im;
    reg [31:0] r_shiftrega = 0;
    reg [31:0] r_shiftregb = 0;
    reg [1:0] r_carry = 0;
    reg [31:0] r_result = 0;
    reg r_bmis = 0;

    /****************************************************************************************/
    reg r_rst;
    always @(posedge w_clk) r_rst <= !w_rst_x | r_halt;

    /*********************************** START_0 ********************************************/
    //update program counter
    wire [17:0] w_pc4 = r_pc + 4;
    wire [17:0] w_pc_true = r_pc + r_imm[17:0];
    wire [17:0] w_npc = (w_bmis) ? w_pc_true : w_pc4;

    assign w_i_addr = r_pc;

    always @(posedge w_clk) begin
        if(r_rst) r_pc <= 0;
        else if(r_pc_we) begin
            r_pc <= w_npc;
            r_pc_we <= 0;
        end
    end

    /****************************************************************************************/
    wire [31:0] w_ir = w_i_in;
    wire [4:0] w_rd,w_rs1,w_rs2;
    wire w_mem_we,w_reg_we,w_op_ld,w_op_im;
    wire [31:0] w_rrs1,w_rrs2;
    wire [9:0] w_alu_ctrl;
    wire [6:0] w_bru_ctrl;
    wire [31:0] w_imm;

    decoder_if decoder_if0(w_ir,w_rd,w_rs1,w_rs2,w_mem_we,w_reg_we,w_op_ld,w_op_im);

    decoder_id decoder_id0(w_ir,w_alu_ctrl,w_bru_ctrl,w_imm);

    regfile regfile0(w_clk,w_rs1,w_rs2,w_rrs1,w_rrs2,1'b1,w_rd,0);
    
    always @(posedge w_clk) begin 
        if(r_state==`START_0) begin
            r_rrs1 <= w_rrs1;
            r_rrs2 <= w_rrs2;
            r_alu_ctrl <= w_alu_ctrl;
            r_bru_ctrl <= w_bru_ctrl;
            r_imm <= w_imm;
            r_mem_we <= w_mem_we;
            r_reg_we <= w_reg_we;
            r_op_ld <= w_op_ld;
            r_op_im <= w_op_im;
            r_pc_we <= 1;
            r_state <= `START_1;
        end
    end
    /*********************************** START_1 ********************************************/
    always@(posedge w_clk) begin
        if(r_state==`START_1) begin
            if(r_cnt[5]==0) begin
                r_shiftrega <= {r_rrs1[0],r_shiftrega[31:1]};
                r_shiftregb <= {r_rrs2[0],r_shiftregb[31:1]};
                r_rrs1 <= {1'b0,r_rrs1[31:1]};
                r_rrs2 <= {1'b0,r_rrs2[31:1]};
                r_cnt <= r_cnt + 1;
            end else begin
                r_shiftrega <= {r_rrs1[0],r_shiftrega[31:1]};
                r_shiftregb <= {r_rrs2[0],r_shiftregb[31:1]};
                r_rrs1 <= {1'b0,r_rrs1[31:1]};
                r_rrs2 <= {1'b0,r_rrs2[31:1]};
                r_state <= `START_2;
                r_cnt <= 1;
            end
        end
    end



    /*********************************** START_2 ********************************************/
    always@(posedge w_clk) begin
        if(r_state==`START_2) begin
            if(r_alu_ctrl!=0 | r_alu_ctrl!=6 | r_alu_ctrl!=7) r_state <= `ALU_0;
            else if(r_alu_ctrl==6) r_state <= `SHIFTL_0;
            else if(r_alu_ctrl==7) r_state <= `SHIFTR_0;
            else if(r_bru_ctrl!=0) r_state <= `BRANCH_0;
        end
    end

    /*********************************** EXECUTION ******************************************/
 
    /*********************************** ALU_0 **********************************************/
    always @(posedge w_clk) begin
        if(r_state==`ALU_0) begin
            if(r_cnt[5]==0) begin 
                case(r_alu_ctrl)
                    1 : begin //add
                            {r_carry,r_shiftrega} <= {r_shiftrega[0] + r_shiftregb[0] + r_carry,r_shiftrega[31:1]};
                        end
                    2 : begin //sub
                            {r_carry,r_shiftrega} <= {r_shiftrega[0] - r_shiftregb[0] - r_carry,r_shiftrega[31:1]};
                        end
                    3 : begin //xor
                            r_shiftrega <= {r_shiftrega[0] ^ r_shiftregb[0],r_shiftrega[31:1]};
                        end
                    4 : begin //or
                            r_shiftrega <= {r_shiftrega[0] | r_shiftregb[0],r_shiftrega[31:1]};
                        end
                    5 : begin //and
                            r_shiftrega <= {r_shiftrega[0] & r_shiftregb[0],r_shiftrega[31:1]};
                        end
                    default : begin 
                            r_shiftrega <= {1'b0,r_shiftrega[31:1]};
                        end
                endcase
                r_shiftregb <= {1'b0,r_shiftregb[31:1]};
                r_cnt <= r_cnt + 1;
            end else begin
                case(r_alu_ctrl)
                    1 : begin //add
                            {r_carry,r_shiftrega} <= {r_shiftrega[0] + r_shiftregb[0] + r_carry,r_shiftrega[31:1]};
                        end
                    2 : begin //sub
                            {r_carry,r_shiftrega} <= {r_shiftrega[0] - r_shiftregb[0] - r_carry,r_shiftrega[31:1]};
                        end
                    3 : begin //xor
                            r_shiftrega <= {r_shiftrega[0] ^ r_shiftregb[0],r_shiftrega[31:1]};
                        end
                    4 : begin //or
                            r_shiftrega <= {r_shiftrega[0] | r_shiftregb[0],r_shiftrega[31:1]};
                        end
                    5 : begin //and
                            r_shiftrega <= {r_shiftrega[0] & r_shiftregb[0],r_shiftrega[31:1]};
                        end
                    default : begin 
                            r_shiftrega <= {1'b0,r_shiftrega[31:1]};
                        end
                endcase
                r_shiftregb <= {1'b0,r_shiftregb[31:1]};
                r_state <= `WRITEBACK_0;
                r_cnt <= 1;
            end
        end
    end    

    /*********************************** BRANCH_0 **********************************************/
    wire w_bmis = (r_bru_ctrl===7'b0000001) ? !r_bmis : r_bmis;

    always @(posedge w_clk) begin
        if(r_state==`BRANCH_0) begin
            if(r_cnt[5]==0) begin
                case(r_bru_ctrl)
                    7'b0000001 : begin //beq
                        if(r_bmis==1'b0) begin
                            r_bmis <= r_bmis + (r_rrs1[0]!=r_rrs2[0]);
                            r_rrs1 <= {1'b0,r_rrs1[31:1]};
                            r_rrs2 <= {1'b0,r_rrs2[31:1]};
                        end else begin 
                            r_rrs1 <= {1'b0,r_rrs1[31:1]};
                            r_rrs2 <= {1'b0,r_rrs2[31:1]};
                        end
                    end
                    7'b0000010 : begin //bne
                        if(r_bmis==1'b0) begin
                            r_bmis <= r_bmis + (r_rrs1[0]==r_rrs2[0]);
                            r_rrs1 <= {1'b0,r_rrs1[31:1]};
                            r_rrs2 <= {1'b0,r_rrs2[31:1]};
                        end else begin
                            r_rrs1 <= {1'b0,r_rrs1[31:1]};
                            r_rrs2 <= {1'b0,r_rrs2[31:1]};
                        end
                    end
                endcase
                r_cnt <= r_cnt + 1;
            end else begin
                case(r_bru_ctrl)
                    7'b0000001 : begin //beq
                        if(r_bmis==1'b0) begin
                            r_bmis <= r_bmis + (r_rrs1[0]!=r_rrs2[0]);
                            r_rrs1 <= {1'b0,r_rrs1[31:1]};
                            r_rrs2 <= {1'b0,r_rrs2[31:1]};
                        end else begin 
                            r_rrs1 <= {1'b0,r_rrs1[31:1]};
                            r_rrs2 <= {1'b0,r_rrs2[31:1]};
                        end
                    end
                    7'b0000010 : begin //bne
                        if(r_bmis==1'b0) begin
                            r_bmis <= r_bmis + (r_rrs1[0]==r_rrs2[0]);
                            r_rrs1 <= {1'b0,r_rrs1[31:1]};
                            r_rrs2 <= {1'b0,r_rrs2[31:1]};
                        end else begin
                            r_rrs1 <= {1'b0,r_rrs1[31:1]};
                            r_rrs2 <= {1'b0,r_rrs2[31:1]};
                        end
                    end
                endcase
                r_state <= `START_0;
                r_cnt <= 1;
            end
        end
    end
    /*********************************** WRITEBACK_0 ****************************************/
    always @(posedge w_clk) begin
        if(r_state==`WRITEBACK_0) begin
            r_result <= r_shiftrega;
            r_shiftrega <= 0;
            r_state <= `START_0;
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
module decoder_if(ir, rd, rs1, rs2, mem_we, reg_we, op_ld, op_imm);
    input  wire [31:0] ir;
    output wire [ 4:0] rd, rs1, rs2;
    output wire        mem_we, reg_we, op_ld, op_imm;
    
    wire [4:0] op = ir[ 6: 2];
    wire r_type = (op==5'b01100);
    wire s_type = (op[4:2]==3'b010); // (op==5'b01000);
    wire b_type = (op==5'b11000);
    wire j_type = (op==5'b11011);
    wire u_type = ({op[4], op[2:0]} ==4'b0101);
    wire i_type = (op==5'b11001 || op==5'b00000 || op==5'b00100);

    assign reg_we = (ir[11:7]!=0) & (op[3:0]!=4'b1000);  //!s_type && !b_type;
    assign mem_we = s_type;
    assign op_ld  = (op==5'b00000);
    assign op_imm = (op==5'b00100);
    assign rd     = (reg_we) ? ir[11:7] : 5'd0;
    assign rs1    = ir[19:15]; // (!u_type && !j_type)       ? ir[19:15] : 5'd0;
    assign rs2    = (!op_imm) ? ir[24:20] : 5'd0;
endmodule

/********************************************************************************************/
module decoder_id(ir, alu_ctrl, bru_ctrl, imm);
    input  wire [31:0] ir;
    output reg  [ 9:0] alu_ctrl;
    output reg  [ 6:0] bru_ctrl;
    output wire [31:0] imm;
    
    wire [4:0] op     = ir[ 6: 2]; // use 5-bit, cause lower 2-bit are always 2'b11
    wire [2:0] funct3 = ir[14:12];
    wire [6:0] funct7 = ir[31:25];

    wire r_type = (op==5'b01100);
    wire s_type = (op[4:2]==3'b010); // (op==5'b01000);
    wire b_type = (op==5'b11000);
    wire j_type = (op==5'b11011);
    wire u_type = ({op[4], op[2:0]} ==4'b0101);
    wire i_type = (op==5'b11001 || op==5'b00000 || op==5'b00100);

    wire [31:0] imm_U = (u_type) ? {ir[31:12], 12'b0} : 0;
    wire [31:0] imm_I = (i_type) ? {{21{ir[31]}}, ir[30:20]} : 0;
    wire [31:0] imm_S = (s_type) ? {{21{ir[31]}}, ir[30:25], ir[11:7]} : 0;
    wire [31:0] imm_B = (b_type) ? {{20{ir[31]}}, ir[7], ir[30:25] ,ir[11:8], 1'b0} : 0;
    wire [31:0] imm_J = (j_type) ? {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0} : 0;
    assign imm = imm_U ^ imm_I ^ imm_S ^ imm_B ^ imm_J;

    reg [3:0] r_alu_ctrl;
    always @(*) begin
        case(op)
            5'b01100 : r_alu_ctrl = {funct7[5], funct3}; 
            5'b00100 : r_alu_ctrl = (funct3==3'h5) ? {funct7[5], funct3} : {1'b0, funct3};
            default  : r_alu_ctrl = 4'b1111;
        endcase
    end

    always @(*) begin
        case(r_alu_ctrl)
            `ALU_CTRL_ADD___ : alu_ctrl = 1;
            `ALU_CTRL_SUB___ : alu_ctrl = 2;
            `ALU_CTRL_XOR___ : alu_ctrl = 3;
            `ALU_CTRL_OR____ : alu_ctrl = 4;
            `ALU_CTRL_AND___ : alu_ctrl = 5;
            `ALU_CTRL_SLL___ : alu_ctrl = 6;
            `ALU_CTRL_SRL___ : alu_ctrl = 7;
            default          : alu_ctrl = 0;
        endcase
    end
    
    always @(*) begin /***** one-hot encoding *****/
        case(op)
            5'b11011 : bru_ctrl =                    7'b1000000;     // JAL  -> taken
            5'b11001 : bru_ctrl =                    7'b1000000;     // JALR -> taken
            5'b11000 : bru_ctrl = (funct3==3'b000) ? 7'b0000001 :    // BEQ
                                  (funct3==3'b001) ? 7'b0000010 :    // BNE
                                  (funct3==3'b100) ? 7'b0000100 :    // BLT
                                  (funct3==3'b101) ? 7'b0001000 :    // BGE
                                  (funct3==3'b110) ? 7'b0010000 :    // BLTU
                                  (funct3==3'b111) ? 7'b0100000 : 0; // BGEU
            default : bru_ctrl = 0;
        endcase
    end
endmodule

/********************************************************************************************/  
module regfile(CLK, rs1, rs2, rdata1, rdata2, WE, rd, wdata);
    input  wire        CLK;
    input  wire [ 4:0] rs1, rs2;
    output wire [31:0] rdata1, rdata2;
    input  wire        WE;
    input  wire [ 4:0] rd;
    input  wire [31:0] wdata;

    reg [31:0] mem [0:31];
    assign rdata1 = (rs1 == 0) ? 0 : (rs1==rd) ? wdata : mem[rs1];
    assign rdata2 = (rs2 == 0) ? 0 : (rs2==rd) ? wdata : mem[rs2];
    always @(posedge CLK) if(WE && (rd!=0)) mem[rd] <= wdata;
endmodule