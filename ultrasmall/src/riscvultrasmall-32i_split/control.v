/********************************************************************************************/
`include "config.vh"
/********************************************************************************************/
`default_nettype none
/********************************************************************************************/
module m_statecounter
    (
        w_clk,
        w_rst,
        w_load,
        w_pll_in;
        w_en,
        r_cnt,
    );
    parameter counter_width = 6;
    parameter init_val = 1;

    input wire w_clk;
    input wire w_rst;
    input wire w_load;
    input wire [counter_width-1:0] w_pll_in;
    input wire w_en;
    output reg [counter_width-1:0] r_cnt;

    always @(posedge w_clk) begin
        if(w_rst) r_cnt <= init_val;
        else if (w_load) r_cnt <= w_pll_in;
        else if (w_en) r_cnt <= r_cnt + 1;
        else r_cnt <= r_cnt;
    end
endmodule

module m_control
    (
        w_clk,
        w_rst,
        w_state_counter;

    );
    input wire w_clk;
    input wire w_rst;
    input wire [4:0] w_state_counter;

    /*********************************** REGISTER *******************************************/

    reg [5:0]       r_state;
    reg [5:0]       r_next_state;
    reg             r_reset_state_counter;


    /*********************************** WIRE ASSIGN*****************************************/


    /***************************************MODULE*******************************************/
    m_statecounter state_counter
        (
            .w_clk(w_clk),
            .w_rst(r_reset_state_counter || w_rst),
            .w_load,
            .w_pll_in;
            .w_en,
            .r_cnt(w_state_counter),
        );
    defparam state_counter.counter_width = 6; //r_cnt is 6bit
    defparam state_counter.init_val      = 1; //r_cnt <= 1:

    /************************************STATE MACHINE***************************************/

    always @(posedge w_clk) begin
        if(w_rst) r_state <= `START_1;
        else r_state <= r_next_state;
    end

    always @(*) begin


        case(r_state)
            `START_1: begin // 2 cycles
                if(w_state_counter[0]) begin
                    r_ctrl_im_en = 1'b1;
                    r_ctrl_bru_reset = 1'b1;
                    r_next_state = `START_1;
                end else begin
                    r_reset_state_counter = 1'b1;
                    r_next_state = `START_2;
                end
            end
            `START_2: begin // 16 cycles
                r_ctrl_bru_en = 1'b1;
                r_crtl_a_shift_en = 1'b1;
                r_crtl_b_shift_en = 1'b1;
                r_crtl_npc_shift_en = 1'b1;
                r_crtl_pc_shift_en = 1'b1;
                r_alu_op = ALU_ADD;

                r_mux_a_shift_in_sel = A_IN_RFA;
                r_mux_b_shift_in_sel = A_IN_RFB;
                r_mux_npc_shift_in_sel = (w_op_branch | w_op_jump) ? NPC_IN_NPC : NPC_IN_ALU;
                r_mux_alu_opA_in_sel = ALU_INA_NPC;
                r_mux_alu_opB_in_sel = ALU_INA_ZERO;

                if(w_state_counter[3:0] == 4'd2) r_ctrl_alu_opB_in_high = 2'b01;

                if(~w_state_counter[4]) begin
                    r_next_state = `START_2;
                end else begin
                    r_reset_state_counter = 1'b1;
                    r_next_state = `START_3;
                end
            end
            `START_3: begin // 1 cycle
                if(w_branch_delay | w_auipc_delay) begin
                  
                end
            end
            `JUMP_1: begin
            end
            `BRANCH_1: begin
            end
            `ALU_1: begin
            end
            `ALUI_1: begin
            end
            `SHIFTR_1: begin
            end
            `SHIFTR_2: begin
            end
            `SHIFTL_1: begin
            end
            `SHIFTL_2: begin
            end
            `LOADSTORE_1: begin
            end
            `LOADSTORE_2: begin
            end
            `STORE_1: begin
            end
            `STORE_2: begin
            end
            `LOAD_1: begin
            end
            `LOAD_2: begin
            end
            `WRITEBACK_1: begin
            end
        endcase
    end




    always @(posedge w_clk) begin
        r_rst <= !w_rst_x;
`ifdef ASYNC_IMEM
        if(r_state==`START_1) begin //1 cycle
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
`else //`ifdef SYNC_IMEM
        if(r_state==`START_1) begin //2 cycle
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
`endif
        end else if(r_state==`START_2) begin //16 cycle
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
        end else if(r_state==`START_3) begin //1 cycle
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
        end else if(r_state==`JUMP_1) begin //16 cycle
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
        end else if(r_state==`BRANCH_1) begin //16 cycle
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
        end else if(r_state==`ALU_1) begin //16 cycle
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
        end else if(r_state==`ALUI_1) begin //16 cycle
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
        end else if(r_state==`SHIFTR_1) begin
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
        end else if(r_state==`SHIFTR_2) begin //17 cycle
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
        end else if(r_state==`SHIFTL_1) begin
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
        end else if(r_state==`SHIFTL_2) begin
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
        end else if(r_state==`LOADSTORE_1) begin
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
`ifdef ASYNC_DMEM
        end else if(r_state==`LOADSTORE_2) begin //1 cycle
            r_shiftregb <= w_d_in;
            r_dmem_offset <= w_dmem_offset; //need reg??
            r_state <= (w_mem_we) ? `STORE_1 : `LOAD_1;
`else
        end else if(r_state==`LOADSTORE_2) begin //2 cycle
            if(r_cnt[0]) begin
                r_d_en <= 0;
                r_cnt <= r_cnt + 1;
            end else begin
                r_shiftregb <= w_d_in;
                r_dmem_offset <= w_dmem_offset; //need reg??
                r_cnt <= 1;
                r_state <= (w_mem_we) ? `STORE_1 : `LOAD_1;
            end
`endif
        end else if(r_state==`STORE_1) begin
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
        end else if(r_state==`STORE_2) begin
`ifdef ASYNC_IMEM
            r_i_en <= 1;
`endif
`ifndef ASYNC_DMEM
            r_d_en <= 0;
`endif
            r_state <= `START_1;
        end else if(r_state==`LOAD_1) begin
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
        end else if(r_state==`LOAD_2) begin
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
        end else if(r_state==`WRITEBACK_1) begin
`ifdef TRACE
            r_valid <= !w_op_jump && !w_op_auipc;
`endif
`ifdef ASYNC_IMEM
            r_i_en <= 1;
`endif
            r_reg_we <= 0;
            r_state <= `START_1;
        end
    end
endmodule