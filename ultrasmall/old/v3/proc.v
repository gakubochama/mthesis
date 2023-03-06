/********************************************************************************************/
`include "config.vh"
/********************************************************************************************/
`default_nettype none
/********************************************************************************************/

`ifdef ASYNC_DMEM
module UltraSmall(w_clk, w_rst_x, w_rout, w_i_addr, w_d_addr, w_i_in, w_d_in, w_d_out, w_i_en, w_d_we);
    input wire w_clk,w_rst_x;
    output wire [31:0] w_rout;
    output wire [31:0] w_i_addr,w_d_addr;
    input wire [31:0] w_i_in,w_d_in;
    output wire [31:0] w_d_out;
    output wire w_i_en;
    output wire [3:0] w_d_we;
`else
module UltraSmall(w_clk, w_rst_x, w_rout, w_i_addr, w_d_addr, w_i_in, w_d_in, w_d_out, w_i_en, w_d_en, w_d_we);
    input wire w_clk,w_rst_x;
    output wire [31:0] w_rout;
    output wire [31:0] w_i_addr,w_d_addr;
    input wire [31:0] w_i_in,w_d_in;
    output wire [31:0] w_d_out;
    output wire w_i_en,w_d_en;
    output wire [3:0] w_d_we;
`endif
    /*********************************** REGISTER *******************************************/
`ifdef IVERILOG
`include "constants.v"
    reg [31:0] r_pc = START_PC_ADDR; //program counter
    reg [31:0] r_npc = START_PC_ADDR + 4; //next program counter
`else
    reg [31:0] r_pc = `START_PC; //program counter
    reg [31:0] r_npc = `START_PC + 4; //next program counter
`endif

`ifdef TRACE
    reg [31:0] r_vpc = 0;
    reg r_valid = 1;//for trace.txt
`endif
    
    reg [4:0] r_state = 0;
    reg [5:0] r_cnt = 1; 
    reg [31:0] r_shiftrega = 0;
    reg [31:0] r_shiftregb = 0;
    reg r_tmp = 0;
    reg r_shiftrega_msb = 0;
    reg r_shiftregb_msb = 0;
    reg r_carry = 0;
    reg [31:0] r_result = 0;
    reg r_i_en = 0;
`ifndef ASYNC_DMEM
    reg r_d_en = 0;
`endif
    reg r_npcadd_en = 0;
    reg r_cal_addr_en = 0;
    reg r_rst = 0;
    reg r_beq = 0,r_bge = 0,r_bgeu = 0;
    reg r_sge = 0,r_sgeu = 0; //for slt,sltu,slti,sltiu
    reg r_bru_result = 0;
    reg r_branch_delay = 0;
    reg r_auipc_delay = 0;
    reg r_pcupdate = 0;
    reg r_reg_we = 0;
    reg [1:0] r_dmem_offset = 0;

    /*********************************** WIRE ASSIGN*****************************************/
    wire [31:0] w_ir = w_i_in;
    wire [4:0] w_rd,w_rs1,w_rs2;
    wire w_mem_we,w_reg_we,w_op_ld,w_op_imm;
    wire [31:0] w_rrs1,w_rrs2;
    wire [6:0] w_op = w_ir[6:0];
    wire [2:0] w_funct3 = w_ir[14:12];
    wire [6:0] w_funct7 = w_ir[31:25];
    wire [9:0] w_alu_ctrl;
    wire [3:0] w_shift_ctrl;
    wire [6:0] w_bru_ctrl;
    wire [31:0] w_imm;
    wire [31:0] w_wdata = w_result;

    //select jump instruction
    wire w_op_jal = w_op[3];
    wire w_op_jalr = ~w_op[3];
    wire w_op_jump = w_bru_ctrl[6];

    //select branch instruction
    wire w_op_beq = w_bru_ctrl[0];
    wire w_op_bne = w_bru_ctrl[1];
    wire w_op_blt = w_bru_ctrl[2];
    wire w_op_bge = w_bru_ctrl[3];
    wire w_op_bltu = w_bru_ctrl[4];
    wire w_op_bgeu = w_bru_ctrl[5];
    wire w_op_branch = w_op_beq ^ w_op_bne ^ w_op_blt ^ w_op_bge ^ w_op_bltu ^ w_op_bgeu;

    //auipc
    wire w_op_auipc = w_alu_ctrl==9;

    //select shift instruction
    wire w_op_sll = w_shift_ctrl[0] & w_op[5];
    wire w_op_slli = w_shift_ctrl[0] & ~w_op[5];
    wire w_op_srl = w_shift_ctrl[1] & ({w_funct7[5],w_op[5]}==2'b01);
    wire w_op_sra = w_shift_ctrl[2] & ({w_funct7[5],w_op[5]}==2'b11);
    wire w_op_srli = w_shift_ctrl[1] & ({w_funct7[5],w_op[5]}==2'b00);
    wire w_op_srai = w_shift_ctrl[2] & ({w_funct7[5],w_op[5]}==2'b10);
    wire w_arithmetic_shift = w_op_sra | w_op_srai;
    wire w_itype_shift = w_op_slli | w_op_srli | w_op_srai;
    wire w_op_shift_left = w_op_sll | w_op_slli;
    wire w_op_shift_right = w_op_srl | w_op_sra | w_op_srli | w_op_srai;

    //select slt instruction
    wire w_op_slt = w_alu_ctrl==6;
    wire w_op_sltu = w_alu_ctrl==7;
    wire w_op_setlessthan = w_op_slt | w_op_sltu;

    //select store instruction
    wire w_op_sb = (w_mem_we && (w_funct3==3'b000));
    wire w_op_sh = (w_mem_we && (w_funct3==3'b001));
    wire w_op_sw = (w_mem_we && (w_funct3==3'b010));

    //select load instruciton
    wire w_op_lb = (w_op_ld && (w_funct3==3'b000));
    wire w_op_lh = (w_op_ld && (w_funct3==3'b001));
    wire w_op_lw = (w_op_ld && (w_funct3==3'b010));
    wire w_op_lbu = (w_op_ld && (w_funct3==3'b100));
    wire w_op_lhu = (w_op_ld && (w_funct3==3'b101));

    //branch resolve
    wire w_beq = r_beq; //x[rs1]==x[rs2]
    wire w_bne = ~r_beq; //x[rs1]!=x[rs2]
    wire w_blt = ~r_bge; //x[rs1]<signed x[rs2]
    wire w_bltu = ~r_bgeu; //x[rs1]<unsigend x[rs2]
    wire w_bge = r_bge; //x[rs1]>=sigend x[rs2]
    wire w_bgeu = r_bgeu;//x[rs1]>=unsigned x[rs2]

    //slt,sltu,slti,sltiu
    wire w_slt = ~r_sge;
    wire w_sltu = ~r_sgeu;

    wire [1:0] w_dmem_offset = (r_state==`LOADSTORE_2) ? r_shiftrega[1:0] : 2'b00;
    wire w_carry;
    wire [1:0] w_alu_result;
    wire [1:0] w_shiftrega_in = f_mux_to_w_shiftrega_in(w_rrs1toshiftreg,w_shiftregb_2,r_shiftrega_msb,r_shiftregb_msb,w_alu_result,w_tmp_result,r_state,w_arithmetic_shift,r_cnt,w_op_lb,w_op_lh,w_op_lw,w_op_lbu,w_op_lhu);
    wire [1:0] w_shiftregb_in = f_mux_to_w_shiftregb_in(w_rrs2toshiftreg,w_imm_2,w_shiftregb_2,r_state,r_cnt,r_dmem_offset,w_op_imm,w_op_sb,w_op_sh,w_op_sw);
    wire w_npcadd_en;
    wire w_cal_addr_en;
    wire [1:0] w_pc_2 = r_pc[1:0];
    wire [1:0] w_npc_2 = r_npc[1:0];
    wire [1:0] w_to_r_npc = ((((w_op_jump & ~r_branch_delay) | (w_op_branch & ~r_branch_delay) | (w_op_auipc & ~r_auipc_delay)) & r_state==`START_2) | (w_op_auipc & r_state==`ALUI_1)) ? w_pc_2 : w_alu_result;
    wire [1:0] w_rrs1toshiftreg = f_mux_to_w_rrs1toshiftreg(w_rrs1,(r_state==`START_2),r_cnt);
    wire [1:0] w_rrs2toshiftreg = f_mux_to_w_rrs2toshiftreg(w_rrs2,(r_state==`START_2 | r_state==`STORE_1),r_cnt,r_state,w_op_sb,w_op_sh);
    wire [1:0] w_imm_2 = f_mux_to_w_imm_2(w_imm,r_cnt);
    wire [1:0] w_shiftrega_2 = r_shiftrega[1:0];
    wire [1:0] w_shiftregb_2 = 
                               (r_state==`LOAD_2) ? 
                               ((r_dmem_offset==2'b00) ? r_shiftregb[1:0] :
                               ((r_dmem_offset==2'b01) ? r_shiftregb[9:8] :
                               ((r_dmem_offset==2'b10) ? r_shiftregb[17:16] :
                               ((r_dmem_offset==2'b11) ? r_shiftregb[25:24] : 2'b00)))) :
                                                        r_shiftregb[1:0];
    wire [1:0] w_tmp_result = f_mux_to_w_tmp_result(r_shiftrega[0],r_shiftrega[31],r_tmp,r_shiftrega_msb,r_state,w_arithmetic_shift,r_cnt[4]);
    wire [1:0] w_alu_in1 = f_mux_to_w_alu_in1(w_shiftrega_2,w_npc_2,r_state,w_op_jal,w_op_auipc);
    wire [1:0] w_alu_in2 = f_mux_to_w_alu_in2(w_shiftregb_2,w_imm_2,{1'b0,r_pcupdate},r_state,r_bru_result);
    wire [31:0] w_result = (r_state==`WRITEBACK_1) ? ((w_op_jump) ? r_pc : ((w_op_setlessthan) ? ((w_op_slt) ? {{31{1'b0}},w_slt} : ((w_op_sltu) ? {{31{1'b0}},w_sltu} : 0)) : r_shiftrega)) : 0;

    assign w_i_addr = r_pc;
    assign w_i_en = r_i_en;
`ifndef ASYNC_DMEM
    assign w_d_en = r_d_en;
`endif
    assign w_npcadd_en = r_npcadd_en;
    assign w_cal_addr_en = r_cal_addr_en;
    assign w_d_we = {4{(w_mem_we && (r_state==`STORE_2))}};
    assign w_d_addr = ((r_state==`LOADSTORE_2 || r_state==`STORE_2 || r_state==`LOAD_2)) ? r_shiftrega : 0;
    assign w_d_out = (r_state==`STORE_2) ? r_shiftregb : 0;

    /***************************************MOODULE******************************************/
    m_decoder decoder0(w_ir,w_rd,w_rs1,w_rs2,w_mem_we,w_reg_we,w_op_ld,w_op_imm,w_alu_ctrl,w_shift_ctrl,w_bru_ctrl,w_imm);

    m_regfile regfile0(w_clk,w_rs1,w_rs2,w_rrs1,w_rrs2,r_reg_we,w_rd,w_wdata);

    m_ALU ALU0(w_alu_in1,w_alu_in2,r_carry,w_alu_ctrl,w_npcadd_en,w_cal_addr_en,w_alu_result,w_carry);

    /************************************REG UPDATE******************************************/
    always @(posedge w_clk) begin
        r_rst <= !w_rst_x;
        case(r_state)
`ifdef ASYNC_IMEM
            `START_1: begin //1 cycle
`ifdef TRACE
                //trace r_pc
                if(r_valid) r_vpc <= r_pc;
                r_valid <= 1'b1;
`endif
                //initialize & reset
                r_shiftrega <= 0;
                r_shiftregb <= 0;
                r_i_en <= 0;
                r_carry <= 0;
                r_npcadd_en <= 1;
                //branch resolve
                r_beq <= 1'b1;
                r_bge <= 1'b1;
                r_bgeu <= 1'b1;
                r_state <= `START_2;
            end
`else //`ifdef SYNC_IMEM
            `START_1: begin //2 cycle
                if(r_cnt[0]) begin
`ifdef TRACE
                    //trace r_pc
                    if(r_valid) r_vpc <= r_pc;
                    r_valid <= 1'b1;
`endif
                    r_i_en <= 1;
                    r_cnt <= r_cnt + 1;
                end else begin
                    r_i_en <= 0;
                    //initialize & reset
                    r_shiftrega <= 0;
                    r_shiftregb <= 0;
                    r_carry <= 0;
                    r_npcadd_en <= 1;
                    //branch resolve
                    r_beq <= 1'b1;
                    r_bge <= 1'b1;
                    r_bgeu <= 1'b1;
                    r_cnt <= 1;
                    r_state <= `START_2;
                end
            end
`endif
            `START_2: begin //16 cycle
                if(r_cnt[4]==0) begin
                    //branch resolve
                    r_beq <= r_beq & (w_rrs1toshiftreg == w_rrs2toshiftreg);
                    r_bge <= (w_rrs1toshiftreg == w_rrs2toshiftreg) ? r_bge : (w_rrs1toshiftreg > w_rrs2toshiftreg);
                    r_bgeu <= (w_rrs1toshiftreg == w_rrs2toshiftreg) ? r_bgeu : (w_rrs1toshiftreg > w_rrs2toshiftreg);

                    if(r_cnt[3:0]==4'd1) r_pcupdate <= 1'b1; //to npc[3:2] + 2'b01 (npc + 4)
                    if(r_cnt[3:0]==4'd2) r_pcupdate <= 1'b0;

                    r_pc <= {w_npc_2,r_pc[31:2]};
                    r_npc <= {w_to_r_npc,r_npc[31:2]}; //if branch or jump inst npc <= pc
                    r_carry <= w_carry;
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= r_cnt + 1;
                end else begin
                    //branch resolve
                    r_beq <= r_beq & (w_rrs1toshiftreg == w_rrs2toshiftreg);
                    r_bge <= (w_rrs1toshiftreg[1] ^ w_rrs2toshiftreg[1]) ? ((w_rrs1toshiftreg[1]==0) ? 1 : 0) : ((w_rrs1toshiftreg[0] == w_rrs2toshiftreg[0]) ? r_bge : (w_rrs1toshiftreg[0] > w_rrs2toshiftreg[0]));
                    r_bgeu <= (w_rrs1toshiftreg == w_rrs2toshiftreg) ? r_bgeu : (w_rrs1toshiftreg > w_rrs2toshiftreg);

                    r_pc <= {w_npc_2,r_pc[31:2]};
                    r_npc <= {w_to_r_npc,r_npc[31:2]}; //if branch or jump inst npc <= pc
                    r_carry <= w_carry;
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= 1; //reset counter
                    r_npcadd_en <= 0;
                    r_state <= `START_3;
                end
            end
            `START_3: begin //1 cycle
                if(r_branch_delay || r_auipc_delay) begin //process delay
`ifdef TRACE
                    r_valid <= 1'b1;
`endif
                    r_branch_delay <= 0;
                    r_auipc_delay <= 0;
`ifdef ASYNC_IMEM
                    r_i_en <= 1;
`endif
                    r_state <= `START_1;
                end
                else if(w_op_jump) begin
                    r_branch_delay <= 1;//next instruction is nop(17 cycle)
                    r_npcadd_en <= 1;
                    r_state <= `JUMP_1;
                end
                else if(w_op_branch) begin
                    r_bru_result <= ^{(w_beq & w_op_beq),(w_bne & w_op_bne),(w_blt & w_op_blt),(w_bge & w_op_bge),(w_bltu & w_op_bltu),(w_bgeu & w_op_bgeu)};
                    r_branch_delay <= 1;//next instruction is nop(17 cycle)
                    r_npcadd_en <= 1; // pc+4 or pc+sext(offset)
                    r_state <= `BRANCH_1;
                end
                else if(w_alu_ctrl!=0) begin 
                    r_auipc_delay <= (w_alu_ctrl==9) ? 1 : 0;
                    r_sge <= 1'b1;
                    r_sgeu <= 1'b1;
                    r_state <= (w_op_imm) ? `ALUI_1 : `ALU_1;
                end 
                else if(w_op_shift_left) begin
                    r_state <= `SHIFTL_1;
                end
                else if(w_op_shift_right) begin
                    r_shiftrega_msb <= r_shiftrega[31];
                    r_state <= `SHIFTR_1;
                end
                else if(w_mem_we | w_op_ld) begin
                    r_state <= `LOADSTORE_1;
                    r_cal_addr_en <= 1;
                end
                r_carry <= 0;
            end
            `JUMP_1: begin //16 cycle
                if(r_cnt[4]==0) begin
                    r_npc <= {w_to_r_npc,r_npc[31:2]}; //npc <= pc + sext(offset)
                    r_carry <= w_carry;
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_cnt <= r_cnt + 1;
                end else begin
                    r_npc <= {w_to_r_npc,r_npc[31:2]}; //npc <= pc + sext(offset)
                    r_carry <= w_carry;
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_cnt <= 1; //reset counter
                    r_npcadd_en <= 0;
                    r_reg_we <= w_reg_we;
                    r_state <= `WRITEBACK_1;
                end
            end
            `BRANCH_1: begin //16 cycle
                if(r_cnt[4]==0) begin

                    if(r_cnt[3:0]==4'd1) r_pcupdate <= 1'b1; //to npc[3:2] + 2'b01 (npc + 4)
                    if(r_cnt[3:0]==4'd2) r_pcupdate <= 1'b0;

                    r_npc <= {w_to_r_npc,r_npc[31:2]}; //npc <= pc + sext(offset) or npc <= pc + 4;
                    r_carry <= w_carry;
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= r_cnt + 1;
                end else begin
`ifdef TRACE
                    r_valid <= 1'b0;
`endif
                    r_npc <= {w_to_r_npc,r_npc[31:2]}; //npc <= pc + sext(offset) or npc <= pc + 4;
                    r_carry <= w_carry;
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= 1; //reset counter
                    r_npcadd_en <= 0;
`ifdef ASYNC_IMEM
                    r_i_en <= 1;
`endif
                    r_state <= `START_1;
                end
            end
            `ALU_1: begin //16 cycle
                if(r_cnt[4]==0) begin 
                    case(w_alu_ctrl)
                        1 : begin //add
                                r_carry <= w_carry;
                            end
                        2 : begin //sub
                                r_carry <= w_carry;
                            end
                        6 : begin //slt
                                r_sge <= (w_alu_in1 == w_alu_in2) ? r_sge : (w_alu_in1 > w_alu_in2);
                            end
                        7 : begin //sltu
                                r_sgeu <= (w_alu_in1 == w_alu_in2) ? r_sgeu : (w_alu_in1 > w_alu_in2);
                            end
                        default : ;
                    endcase
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= r_cnt + 1;
                end else begin
                    case(w_alu_ctrl)
                        1 : begin //add
                                r_carry <= w_carry;
                            end
                        2 : begin //sub
                                r_carry <= w_carry;
                            end
                        6 : begin //slt
                                r_sge <= (w_alu_in1[1] ^ w_alu_in2[1]) ? ((w_alu_in1[1]==0) ? 1 : 0) : ((w_alu_in1[0] == w_alu_in2[0]) ? r_sge : (w_alu_in1[0] > w_alu_in2[0]));
                            end
                        7 : begin //sltu
                                r_sgeu <= (w_alu_in1 == w_alu_in2) ? r_sgeu : (w_alu_in1 > w_alu_in2);
                            end
                        default : ;
                    endcase
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_reg_we <= w_reg_we;
                    r_state <= `WRITEBACK_1;
                    r_cnt <= 1;
                end
            end
            `ALUI_1: begin //16 cycle
                if(r_cnt[4]==0) begin 
                    case(w_alu_ctrl)
                        1 : begin //addi
                                r_carry <= w_carry;
                            end
                        6 : begin //slti
                                r_sge <= (w_alu_in1 == w_alu_in2) ? r_sge : (w_alu_in1 > w_alu_in2);
                            end
                        7 : begin //sltiu
                                r_sgeu <= (w_alu_in1 == w_alu_in2) ? r_sgeu : (w_alu_in1 > w_alu_in2);
                            end
                        8 : begin //lui
                                r_carry <= w_carry;
                            end
                        9 : begin //auipc
                                //reset pc & npc
                                r_pc <= {w_npc_2,r_pc[31:2]};
                                r_npc <= {w_to_r_npc,r_npc[31:2]};
                                r_carry <= w_carry;
                            end
                        default : ;
                    endcase
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= r_cnt + 1;
                end else begin
                    case(w_alu_ctrl)
                        1 : begin //addi
                                r_carry <= w_carry;
                            end
                        6 : begin //slt
                                r_sge <= (w_alu_in1[1] ^ w_alu_in2[1]) ? ((w_alu_in1[1]==0) ? 1 : 0) : ((w_alu_in1[0] == w_alu_in2[0]) ? r_sge : (w_alu_in1[0] > w_alu_in2[0]));
                            end
                        7 : begin //sltu
                                r_sgeu <= (w_alu_in1 == w_alu_in2) ? r_sgeu : (w_alu_in1 > w_alu_in2);
                            end
                        8 : begin //lui
                                r_carry <= w_carry;
                            end
                        9 : begin //auipc
                                //reset pc & npc
                                r_pc <= {w_npc_2,r_pc[31:2]};
                                r_npc <= {w_to_r_npc,r_npc[31:2]};
                                r_carry <= w_carry;
                            end
                        default : ;
                    endcase
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_reg_we <= w_reg_we;
                    r_state <= `WRITEBACK_1;
                    r_cnt <= 1;
                end
            end
            `SHIFTR_1: begin
                case(r_shiftregb[4:0])
                    1 : begin   
                            r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    2 : begin              
                            r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                            r_reg_we <= w_reg_we;                
                            r_state <= `WRITEBACK_1;                   
                        end
                    3 : begin
                            r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                            r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    4 : begin
                            if(r_cnt!=2) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    5 : begin
                            if(r_cnt!=2) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    6 : begin
                            if(r_cnt!=3) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    7 : begin
                            if(r_cnt!=3) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    8 : begin
                            if(r_cnt!=4) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    9 : begin
                            if(r_cnt!=4) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    10 : begin
                            if(r_cnt!=5) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    11 : begin
                            if(r_cnt!=5) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    12 : begin
                            if(r_cnt!=6) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    13 : begin
                            if(r_cnt!=6) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    14 : begin
                            if(r_cnt!=7) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    15 : begin
                            if(r_cnt!=7) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    16 : begin
                            if(r_cnt!=8) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    17 : begin
                            if(r_cnt!=8) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    18 : begin
                            if(r_cnt!=9) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    19 : begin
                            if(r_cnt!=9) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    20 : begin
                            if(r_cnt!=10) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    21 : begin
                            if(r_cnt!=10) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    22 : begin
                            if(r_cnt!=11) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    23 : begin
                            if(r_cnt!=11) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    24 : begin
                            if(r_cnt!=12) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    25 : begin
                            if(r_cnt!=12) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    26 : begin
                            if(r_cnt!=13) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    27 : begin
                            if(r_cnt!=13) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    28 : begin
                            if(r_cnt!=14) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    29 : begin
                            if(r_cnt!=14) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    30 : begin
                            if(r_cnt!=15) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    31 : begin
                            if(r_cnt!=15) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= (w_arithmetic_shift) ? r_shiftrega_msb : 1'b0;
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 0;
                                r_state <= `SHIFTR_2;
                            end
                        end
                    32 : begin
                            if(r_cnt==15) begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    default : begin
                        r_reg_we <= w_reg_we;
                        r_state <= `WRITEBACK_1;
                    end
                endcase
            end
            `SHIFTR_2: begin //17 cycle
                if(r_cnt[4]==0) begin
                    r_tmp <= r_shiftrega[1];
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]}; 
                    r_cnt <= r_cnt + 1;
                end else begin
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};  
                    r_cnt <= 1;
                    r_reg_we <= w_reg_we;
                    r_state <= `WRITEBACK_1;
                end
            end
            `SHIFTL_1: begin
                case(r_shiftregb[4:0])
                    1 : begin
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    2 : begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};
                            r_reg_we <= w_reg_we;
                            r_state <= `WRITEBACK_1;
                        end
                    3 : begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    4 : begin
                            if(r_cnt!=2) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    5 : begin
                            if(r_cnt!=2) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    6 : begin
                            if(r_cnt!=3) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    7 : begin
                            if(r_cnt!=3) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    8 : begin
                            if(r_cnt!=4) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    9 : begin
                            if(r_cnt!=4) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    10 : begin
                            if(r_cnt!=5) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    11 : begin
                            if(r_cnt!=5) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    12 : begin
                            if(r_cnt!=6) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    13 : begin
                            if(r_cnt!=6) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    14 : begin
                            if(r_cnt!=7) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    15 : begin
                            if(r_cnt!=7) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    16 : begin
                            if(r_cnt!=8) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    17 : begin
                            if(r_cnt!=8) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    18 : begin
                            if(r_cnt!=9) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    19 : begin
                            if(r_cnt!=9) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    20 : begin
                            if(r_cnt!=10) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    21 : begin
                            if(r_cnt!=10) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    22 : begin
                            if(r_cnt!=11) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    23 : begin
                            if(r_cnt!=11) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    24 : begin
                            if(r_cnt!=12) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    25 : begin
                            if(r_cnt!=12) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    26 : begin
                            if(r_cnt!=13) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    27 : begin
                            if(r_cnt!=13) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    28 : begin
                            if(r_cnt!=14) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    29 : begin
                            if(r_cnt!=14) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    30 : begin
                            if(r_cnt!=15) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    31 : begin
                            if(r_cnt!=15) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 0;
                                r_state <= `SHIFTL_2;
                            end
                        end
                    32 : begin
                            if(r_cnt==15) begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= r_cnt + 1;
                            end else begin
                                r_tmp <= r_shiftrega[30];
                                r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                                r_cnt <= 1;
                                r_reg_we <= w_reg_we;
                                r_state <= `WRITEBACK_1;
                            end
                        end
                    default : begin
                        r_reg_we <= w_reg_we;
                        r_state <= `WRITEBACK_1;
                    end
                endcase
            end
            `SHIFTL_2: begin
                if(r_cnt[4]==0) begin
                    r_tmp <= r_shiftrega[30];
                    r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                    r_cnt <= r_cnt + 1;
                end else begin
                    r_tmp <= r_shiftrega[30];
                    r_shiftrega <= {r_shiftrega[29:0],w_shiftrega_in};  
                    r_cnt <= 1;
                    r_reg_we <= w_reg_we;
                    r_state <= `WRITEBACK_1;
                end
            end
            `LOADSTORE_1: begin
                if(r_cnt[4]==0) begin
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_carry <= w_carry;
                    r_cnt <= r_cnt + 1;
                end else begin
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_carry <= w_carry;
                    r_cnt <= 1;
                    r_cal_addr_en <= 0;
`ifndef ASYNC_DMEM
                    r_d_en <= 1;
`endif
                    r_state <= `LOADSTORE_2;
                end
            end
`ifdef ASYNC_DMEM
            `LOADSTORE_2: begin //1 cycle
                r_shiftregb <= w_d_in;
                r_dmem_offset <= w_dmem_offset; //need reg??
                r_state <= (w_mem_we) ? `STORE_1 : `LOAD_1;
            end
`else
            `LOADSTORE_2: begin //2 cycle
                if(r_cnt[0]) begin
                    r_d_en <= 0;
                    r_cnt <= r_cnt + 1;
                end else begin
                    r_shiftregb <= w_d_in;
                    r_dmem_offset <= w_dmem_offset; //need reg??
                    r_cnt <= 1;
                    r_state <= (w_mem_we) ? `STORE_1 : `LOAD_1;
                end
            end
`endif
            `STORE_1: begin
                if(r_cnt[4]==0) begin
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= r_cnt + 1;
                end else begin
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= 1;
`ifndef ASYNC_DMEM
                    r_d_en <= 1;
`endif
                    r_state <= `STORE_2;
                end
            end
            `STORE_2: begin
`ifdef ASYNC_IMEM
                r_i_en <= 1;
`endif
`ifndef ASYNC_DMEM
                r_d_en <= 0;
`endif
                r_state <= `START_1;
            end
            `LOAD_1: begin
                r_shiftregb_msb <= (w_op_lb) ? 
                ((r_dmem_offset==2'b00) ? r_shiftregb[7] :
                ((r_dmem_offset==2'b01) ? r_shiftregb[15] :
                ((r_dmem_offset==2'b10) ? r_shiftregb[23] :
                r_shiftregb[31]))) : 
                ((w_op_lh) ? 
                ((r_dmem_offset==2'b00) ? r_shiftregb[15] : 
                r_shiftregb[31]) : 1'b0);
`ifndef ASYNC_DMEM
                r_d_en <= 1;
`endif
                r_state <= `LOAD_2;
            end
            `LOAD_2: begin
                if(r_cnt[4]==0) begin
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= r_cnt + 1;
                end else begin
                    r_shiftrega <= {w_shiftrega_in,r_shiftrega[31:2]};
                    r_shiftregb <= {w_shiftregb_in,r_shiftregb[31:2]};
                    r_cnt <= 1;
                    r_reg_we <= w_reg_we;
                    r_state <= `WRITEBACK_1;
                end
`ifndef ASYNC_DMEM
                r_d_en <= 0;
`endif
            end
            `WRITEBACK_1: begin
`ifdef TRACE
                r_valid <= !w_op_jump && !w_op_auipc;
`endif
`ifdef ASYNC_IMEM
                r_i_en <= 1;
`endif
                r_reg_we <= 0;
                r_state <= `START_1;
            end
        endcase
    end


    /*********************************** FUNCTIONS ******************************************/
    function [1:0] f_mux_to_w_imm_2;
        input [31:0] w_imm;
        input [5:0] w_sel;

        begin
            case(w_sel[4:0])
                1 : f_mux_to_w_imm_2 = w_imm[1:0];
                2 : f_mux_to_w_imm_2 = w_imm[3:2];
                3 : f_mux_to_w_imm_2 = w_imm[5:4];
                4 : f_mux_to_w_imm_2 = w_imm[7:6];
                5 : f_mux_to_w_imm_2 = w_imm[9:8];
                6 : f_mux_to_w_imm_2 = w_imm[11:10];
                7 : f_mux_to_w_imm_2 = w_imm[13:12];
                8 : f_mux_to_w_imm_2 = w_imm[15:14];
                9 : f_mux_to_w_imm_2 = w_imm[17:16];
                10 : f_mux_to_w_imm_2 = w_imm[19:18];
                11 : f_mux_to_w_imm_2 = w_imm[21:20];
                12 : f_mux_to_w_imm_2 = w_imm[23:22];
                13 : f_mux_to_w_imm_2 = w_imm[25:24];
                14 : f_mux_to_w_imm_2 = w_imm[27:26];
                15 : f_mux_to_w_imm_2 = w_imm[29:28];
                16 : f_mux_to_w_imm_2 = w_imm[31:30];
                default : f_mux_to_w_imm_2 = 2'b00;
            endcase
        end
    endfunction

    function [1:0] f_mux_to_w_rrs1toshiftreg; //34:2 mux
        input [31:0] w_in;
        input w_en;
        input [5:0] w_sel;
        
        begin
            if(w_en) begin
                case(w_sel[3:0])
                    1 : f_mux_to_w_rrs1toshiftreg = w_in[1:0];
                    2 : f_mux_to_w_rrs1toshiftreg = w_in[3:2];
                    3 : f_mux_to_w_rrs1toshiftreg = w_in[5:4];
                    4 : f_mux_to_w_rrs1toshiftreg = w_in[7:6];
                    5 : f_mux_to_w_rrs1toshiftreg = w_in[9:8];
                    6 : f_mux_to_w_rrs1toshiftreg = w_in[11:10];
                    7 : f_mux_to_w_rrs1toshiftreg = w_in[13:12];
                    8 : f_mux_to_w_rrs1toshiftreg = w_in[15:14];
                    9 : f_mux_to_w_rrs1toshiftreg = w_in[17:16];
                    10 : f_mux_to_w_rrs1toshiftreg = w_in[19:18];
                    11 : f_mux_to_w_rrs1toshiftreg = w_in[21:20];
                    12 : f_mux_to_w_rrs1toshiftreg = w_in[23:22];
                    13 : f_mux_to_w_rrs1toshiftreg = w_in[25:24];
                    14 : f_mux_to_w_rrs1toshiftreg = w_in[27:26];
                    15 : f_mux_to_w_rrs1toshiftreg = w_in[29:28];
                    default : f_mux_to_w_rrs1toshiftreg = w_in[31:30];
                endcase
            end
        end
    endfunction

    function [1:0] f_mux_to_w_rrs2toshiftreg; //34:2 mux
        input [31:0] w_in;
        input w_en;
        input [5:0] w_sel;
        input [5:0] w_state;
        input w_op_sb;
        input w_op_sh;
        
        begin
            if(w_en) begin
                if(w_op_sb & (w_state==`STORE_1)) begin //for store byte
                    case(w_sel[1:0])
                        1 : f_mux_to_w_rrs2toshiftreg = w_in[1:0];
                        2 : f_mux_to_w_rrs2toshiftreg = w_in[3:2];
                        3 : f_mux_to_w_rrs2toshiftreg = w_in[5:4];
                        default : f_mux_to_w_rrs2toshiftreg = w_in[7:6];
                    endcase
                end else if(w_op_sh & (w_state==`STORE_1)) begin //for store half
                    case(w_sel[2:0])
                        1 : f_mux_to_w_rrs2toshiftreg = w_in[1:0];
                        2 : f_mux_to_w_rrs2toshiftreg = w_in[3:2];
                        3 : f_mux_to_w_rrs2toshiftreg = w_in[5:4];
                        4 : f_mux_to_w_rrs2toshiftreg = w_in[7:6];
                        5 : f_mux_to_w_rrs2toshiftreg = w_in[9:8];
                        6 : f_mux_to_w_rrs2toshiftreg = w_in[11:10];
                        7 : f_mux_to_w_rrs2toshiftreg = w_in[13:12];
                        default : f_mux_to_w_rrs2toshiftreg = w_in[15:14];
                    endcase
                end else begin
                    case(w_sel[3:0])
                        1 : f_mux_to_w_rrs2toshiftreg = w_in[1:0];
                        2 : f_mux_to_w_rrs2toshiftreg = w_in[3:2];
                        3 : f_mux_to_w_rrs2toshiftreg = w_in[5:4];
                        4 : f_mux_to_w_rrs2toshiftreg = w_in[7:6];
                        5 : f_mux_to_w_rrs2toshiftreg = w_in[9:8];
                        6 : f_mux_to_w_rrs2toshiftreg = w_in[11:10];
                        7 : f_mux_to_w_rrs2toshiftreg = w_in[13:12];
                        8 : f_mux_to_w_rrs2toshiftreg = w_in[15:14];
                        9 : f_mux_to_w_rrs2toshiftreg = w_in[17:16];
                        10 : f_mux_to_w_rrs2toshiftreg = w_in[19:18];
                        11 : f_mux_to_w_rrs2toshiftreg = w_in[21:20];
                        12 : f_mux_to_w_rrs2toshiftreg = w_in[23:22];
                        13 : f_mux_to_w_rrs2toshiftreg = w_in[25:24];
                        14 : f_mux_to_w_rrs2toshiftreg = w_in[27:26];
                        15 : f_mux_to_w_rrs2toshiftreg = w_in[29:28];
                        default : f_mux_to_w_rrs2toshiftreg = w_in[31:30];
                    endcase
                end
            end
        end
    endfunction

    function [1:0] f_mux_to_w_shiftrega_in;
        input [1:0] w_rrs1toshiftreg;
        //input [1:0] w_shiftrega_2;
        input [1:0] w_shiftregb_2;
        input w_shiftrega_msb;
        input w_shiftregb_msb;
        input [1:0] w_alu_result;
        input [1:0] w_tmp_result;
        input [4:0] w_state;
        input w_arithmetic_shift;
        input [5:0] w_cnt;
        input w_op_lb,w_op_lh,w_op_lw,w_op_lbu,w_op_lhu;

        begin
            case(w_state)
                `START_2 : f_mux_to_w_shiftrega_in = w_rrs1toshiftreg;
                `ALU_1 : f_mux_to_w_shiftrega_in = w_alu_result;
                `ALUI_1 : f_mux_to_w_shiftrega_in = w_alu_result;
                `SHIFTR_1 : f_mux_to_w_shiftrega_in = (w_arithmetic_shift) ? {w_shiftrega_msb,w_shiftrega_msb} : 2'b00;
                `SHIFTL_1 : f_mux_to_w_shiftrega_in = 2'b00;
                `SHIFTR_2 : f_mux_to_w_shiftrega_in = w_tmp_result;
                `SHIFTL_2 : f_mux_to_w_shiftrega_in = w_tmp_result;
                `LOADSTORE_1 : f_mux_to_w_shiftrega_in = w_alu_result;
                `LOAD_2 : begin
                    if(w_op_lb) begin //lb
                    f_mux_to_w_shiftrega_in = (w_cnt<6'd5) ? w_shiftregb_2 : {w_shiftregb_msb,w_shiftregb_msb};
                    end
                    else if(w_op_lh) begin //lh
                    f_mux_to_w_shiftrega_in = (w_cnt<6'd9) ? w_shiftregb_2 : {w_shiftregb_msb,w_shiftregb_msb};
                    end
                    else if(w_op_lbu) begin //lbu
                    f_mux_to_w_shiftrega_in = (w_cnt<6'd5) ? w_shiftregb_2 : 2'b00;
                    end
                    else if(w_op_lhu) begin //lhu
                    f_mux_to_w_shiftrega_in = (w_cnt<6'd9) ? w_shiftregb_2 : 2'b00;
                    end
                    else f_mux_to_w_shiftrega_in = w_shiftregb_2; //lw
                end
                default : f_mux_to_w_shiftrega_in = 2'b00;
            endcase
        end
    endfunction

    function [1:0] f_mux_to_w_shiftregb_in;
        input [1:0] w_rrs2toshiftreg;
        input [1:0] w_imm_2;
        input [1:0] w_shiftregb_2;
        input [4:0] w_state;
        input [5:0] w_cnt;
        input [1:0] w_dmem_offset;
        input w_op_imm,w_op_sb,w_op_sh,w_op_sw;

        begin
            case(w_state)
                `START_2 : f_mux_to_w_shiftregb_in = (w_op_imm) ? w_imm_2 : w_rrs2toshiftreg;
                `STORE_1 : begin
                    if(w_op_sb) begin //sb
                        f_mux_to_w_shiftregb_in = 
                        (w_dmem_offset==2'b00) ? ((w_cnt>6'd4) ? w_shiftregb_2 : w_rrs2toshiftreg) :
                        (w_dmem_offset==2'b01) ? ((w_cnt<6'd5 | w_cnt>6'd8) ? w_shiftregb_2 : w_rrs2toshiftreg) :
                        (w_dmem_offset==2'b10) ? ((w_cnt<6'd9 | w_cnt>6'd12) ? w_shiftregb_2 : w_rrs2toshiftreg) :
                                                 ((w_cnt<6'd13) ? w_shiftregb_2 : w_rrs2toshiftreg);
                    end
                    else if(w_op_sh) begin //sh
                        f_mux_to_w_shiftregb_in = 
                        (w_dmem_offset==2'b00) ? ((w_cnt>6'd8) ? w_shiftregb_2 : w_rrs2toshiftreg) :
                                                 ((w_cnt<6'd9) ? w_shiftregb_2 : w_rrs2toshiftreg);
                    end
                    else begin
                        f_mux_to_w_shiftregb_in = w_rrs2toshiftreg; //sw
                    end
                end
                `LOAD_2 : f_mux_to_w_shiftregb_in = w_shiftregb_2;
                default : f_mux_to_w_shiftregb_in = 2'b00;
            endcase
        end
    endfunction

    function [1:0] f_mux_to_w_tmp_result;
        input w_shiftreg_0;
        input w_shiftreg_31;
        input w_tmp;
        input w_shiftrega_msb;
        input [4:0] w_state;
        input w_arithmetic_shift;
        input w_cnt_4;
        
        begin  
            case(w_state)
                `SHIFTR_2 : f_mux_to_w_tmp_result = (w_cnt_4==1) ? {(w_arithmetic_shift) ? w_shiftrega_msb: 1'b0,w_tmp} : {w_shiftreg_0,w_tmp};
                `SHIFTL_2 : f_mux_to_w_tmp_result = (w_cnt_4==1) ? {w_tmp,1'b0} : {w_tmp,w_shiftreg_31};
                default : f_mux_to_w_tmp_result = 2'b00;
            endcase
        end
    endfunction

    function [1:0] f_mux_to_w_alu_in1;
        input [1:0] w_shiftrega_2;
        input [1:0] w_npc_2;
        input [4:0] w_state;
        input w_op_jal;
        input w_op_auipc;

        begin
            case(w_state)
                `START_2 : f_mux_to_w_alu_in1 = w_npc_2;
                `JUMP_1 : f_mux_to_w_alu_in1 = (w_op_jal) ? w_npc_2 : w_shiftrega_2;
                `BRANCH_1 : f_mux_to_w_alu_in1 = w_npc_2;
                `ALUI_1 : f_mux_to_w_alu_in1 = (w_op_auipc) ? w_npc_2 : w_shiftrega_2;
                default : f_mux_to_w_alu_in1 = w_shiftrega_2;
            endcase
        end

    endfunction

    function [1:0] f_mux_to_w_alu_in2;
        input [1:0] w_shiftregb_2;
        input [1:0] w_imm_2;
        input [1:0] w_pcupdate;
        input [4:0] w_state;
        input w_bru_result;
        
        begin
            case(w_state)
                `START_2 : f_mux_to_w_alu_in2 = w_pcupdate;
                `JUMP_1 : f_mux_to_w_alu_in2 = w_imm_2;
                `BRANCH_1 : f_mux_to_w_alu_in2 = (w_bru_result) ? w_imm_2 : w_pcupdate;
                `ALU_1 : f_mux_to_w_alu_in2 = w_shiftregb_2;
                `ALUI_1 : f_mux_to_w_alu_in2 = w_imm_2;
                `LOADSTORE_1 : f_mux_to_w_alu_in2 = w_imm_2;
                default : f_mux_to_w_alu_in2 = w_shiftregb_2;
            endcase
        end

    endfunction

endmodule


/********************************************************************************************/
module m_decoder(w_ir,w_rd,w_rs1,w_rs2,w_mem_we,w_reg_we,w_op_ld,w_op_imm,r_alu_ctrl,r_shift_ctrl,r_bru_ctrl,w_imm);
    input wire [31:0] w_ir;
    output wire [4:0] w_rd,w_rs1,w_rs2;
    output wire [31:0] w_imm;
    output wire w_mem_we,w_reg_we,w_op_ld,w_op_imm;
    output reg [9:0] r_alu_ctrl;
    output reg [3:0] r_shift_ctrl;
    output reg [6:0] r_bru_ctrl;

    //select format
    wire [4:0] w_op = w_ir[6:2]; // use 5-bit, because lower 2-bit are always 2'b11
    wire [2:0] w_funct3 = w_ir[14:12];
    wire [6:0] w_funct7 = w_ir[31:25];
    wire [4:0] w_shamt = w_ir[24:20]; //for shift instruction

    wire w_r_type = (w_op==5'b01100);
    wire w_s_type = (w_op[4:2]==3'b010); // (w_op==5'b01000);
    wire w_b_type = (w_op==5'b11000); //jump
    wire w_j_type = (w_op==5'b11011); //branch
    wire w_u_type = ({w_op[4], w_op[2:0]} ==4'b0101); //lui,auipc
    wire w_i_type = (w_op==5'b11001 || w_op==5'b00000 || w_op==5'b00100);

    wire [31:0] w_imm_U = (w_u_type) ? {w_ir[31:12], 12'b0} : 0;
    wire [31:0] w_imm_I = (w_i_type && ({w_funct3,w_op}==4'b1011 || {w_funct3,w_op[2]}==4'b0011)) ? {{27{1'b0}},w_shamt} : ((~w_i_type) ? 0 : {{21{w_ir[31]}}, w_ir[30:20]});
    wire [31:0] w_imm_S = (w_s_type) ? {{21{w_ir[31]}}, w_ir[30:25], w_ir[11:7]} : 0;
    wire [31:0] w_imm_B = (w_b_type) ? {{20{w_ir[31]}}, w_ir[7], w_ir[30:25] ,w_ir[11:8], 1'b0} : 0;
    wire [31:0] w_imm_J = (w_j_type) ? {{12{w_ir[31]}}, w_ir[19:12], w_ir[20], w_ir[30:21], 1'b0} : 0;

    assign w_reg_we = (w_ir[11:7]!=0) & (w_op[3:0]!=4'b1000);  //!w_s_type && !w_b_type;
    assign w_mem_we = w_s_type;
    assign w_op_ld  = (w_op==5'b00000);
    assign w_op_imm = (w_op==5'b00100) | w_u_type;
    assign w_rd     = (w_reg_we) ? w_ir[11:7] : 5'd0;
    assign w_rs1    = (!w_u_type && !w_j_type) ? w_ir[19:15] : 5'd0;
    assign w_rs2    = (!w_op_imm) ? w_ir[24:20] : 5'd0;
    assign w_imm = w_imm_U ^ w_imm_I ^ w_imm_S ^ w_imm_B ^ w_imm_J;

    reg [3:0] r_alu_ctrl0;
    always @(*) begin
        case(w_op)
            5'b01100 : r_alu_ctrl0 = {w_funct7[5], w_funct3}; 
            5'b00100 : r_alu_ctrl0 = (w_funct3==3'h5) ? {w_funct7[5], w_funct3} : {1'b0, w_funct3};
            5'b01101 : r_alu_ctrl0 = 4'h9;
            5'b00101 : r_alu_ctrl0 = 4'hA;
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
            `ALU_CTRL_SLT___ : r_alu_ctrl = 6;
            `ALU_CTRL_SLTU__ : r_alu_ctrl = 7;
            `ALU_CTRL_LUI___ : r_alu_ctrl = 8;
            `ALU_CTRL_AUIPC_ : r_alu_ctrl = 9;
            default          : r_alu_ctrl = 0;
        endcase
    end

    always @(*) begin
        case(r_alu_ctrl0)
            `SHIFT_CTRL_SLL___ : r_shift_ctrl = 4'b0001;
            `SHIFT_CTRL_SRL___ : r_shift_ctrl = 4'b0010;
            `SHIFT_CTRL_SRA___ : r_shift_ctrl = 4'b0100;
            default            : r_shift_ctrl = 4'b0000;
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

module m_regfile(w_clk, w_rs1, w_rs2, w_rdata1, w_rdata2, w_we, w_rd, w_wdata);
    input  wire        w_clk;
    input  wire [ 4:0] w_rs1, w_rs2;
    output wire [31:0] w_rdata1, w_rdata2;
    input  wire        w_we;
    input  wire [ 4:0] w_rd;
    input  wire [31:0] w_wdata;

    reg [31:0] mem [0:31];
    integer i; initial for (i=0; i<32; i=i+1) mem[i] = 0;
    assign w_rdata1 = mem[w_rs1];
    assign w_rdata2 = mem[w_rs2];
    always @(posedge w_clk) if(w_we && (w_rd!=0)) mem[w_rd] <= w_wdata;
endmodule

/********************************************************************************************/  

module m_ALU(w_alu_in1,w_alu_in2,w_carry_in,w_alu_ctrl,w_npcadd_en,w_cal_addr_en,w_alu_result,w_carry_out);
    input wire [1:0] w_alu_in1,w_alu_in2;
    input wire w_carry_in;
    input wire [9:0] w_alu_ctrl;
    input wire w_npcadd_en;
    input wire w_cal_addr_en;
    output wire [1:0] w_alu_result;
    output wire w_carry_out;

    reg [1:0] r_alu_result;
    reg r_carry_out;

    always @(*) begin
        if(w_npcadd_en) begin
            {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} + {1'b0,w_alu_in2} + {2'b0,w_carry_in}; //add
        end else if(w_cal_addr_en) begin
            {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} + {1'b0,w_alu_in2} + {2'b0,w_carry_in}; //add
        end else begin
            case(w_alu_ctrl)
                1 : {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} + {1'b0,w_alu_in2} + {2'b0,w_carry_in}; //add
                2 : {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} - {1'b0,w_alu_in2} - {2'b0,w_carry_in}; //sub
                3 : r_alu_result = w_alu_in1 ^ w_alu_in2; //xor
                4 : r_alu_result = w_alu_in1 | w_alu_in2; //or
                5 : r_alu_result = w_alu_in1 & w_alu_in2; //and
                6 : r_alu_result = 2'b00; //slt
                7 : r_alu_result = 2'b00; //sltu
                8 : {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} + {1'b0,w_alu_in2} + {2'b0,w_carry_in}; //lui
                9 : {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} + {1'b0,w_alu_in2} + {2'b0,w_carry_in}; //auipc
                default : r_alu_result = 0;
            endcase
        end
    end

    assign w_alu_result = r_alu_result;
    assign w_carry_out = r_carry_out;
endmodule