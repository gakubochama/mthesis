/********************************************************************************************/
`include "config.vh"
/********************************************************************************************/
`default_nettype none
/********************************************************************************************/

module UltraSmall(w_clk, w_rst_x, r_rout, r_halt, w_i_addr, w_d_addr, w_i_in, w_d_in, w_d_out, w_i_en, w_d_we);
    input wire w_clk,w_rst_x;
    output reg [31:0] r_rout;
    output reg r_halt;
    output wire [31:0] w_i_addr,w_d_addr;
    input wire [31:0] w_i_in,w_d_in;
    output wire [31:0] w_d_out;
    output wire w_i_en;
    output wire [3:0] w_d_we;

    /*********************************** Register *******************************************/
    reg [31:0] r_pc = 0; //program counter
    reg [31:0] r_npc = 32'd4; //program counter
    reg [4:0] r_state = 0;
    reg [5:0] r_cnt = 1; 
    reg [31:0] r_shiftrega = 0;
    reg [31:0] r_shiftregb = 32'd4;
    reg r_tmp;
    reg r_carry = 0;
    reg [31:0] r_result = 0;
    reg r_bmis = 0;
    reg r_i_en = 0;
    reg r_stall = 0;
    reg r_npcadd_en = 0;
    reg r_rst;

    /****************************************************************************************/
    always @(posedge w_clk) r_rst <= !w_rst_x | r_halt;

    /*********************************** START_1 ********************************************/
    always @(posedge w_clk) begin
        if(r_state==`START_1) r_i_en <= 0;
    end

    /****************************************************************************************/
    wire [31:0] w_ir;
    wire [4:0] w_rd,w_rs1,w_rs2;
    wire w_mem_we,w_reg_we,w_op_ld,w_op_imm;
    wire [31:0] w_rrs1,w_rrs2;
    wire [9:0] w_alu_ctrl;
    wire [3:0] w_shift_ctrl;
    wire [6:0] w_bru_ctrl;
    wire [31:0] w_imm;

    m_decoder decoder0(w_ir,w_rd,w_rs1,w_rs2,w_mem_we,w_reg_we,w_op_ld,w_op_imm,w_alu_ctrl,w_shift_ctrl,w_bru_ctrl,w_imm);

    m_regfile regfile0(w_clk,w_rs1,w_rs2,w_rrs1,w_rrs2,w_reg_we,w_rd,w_result);
    
    always @(posedge w_clk) begin 
        if(r_state==`START_1) begin
            r_carry <= 0;
            r_npcadd_en <= 1;
            r_state <= `START_2;
        end
    end
    /*********************************** START_2 ********************************************/
    wire w_carry;

    always@(posedge w_clk) begin
        if(r_state==`START_2) begin
            if(r_cnt[4]==0) begin
                r_pc <= {r_npc[1:0],r_pc[31:2]};
                r_npc <= {w_to_r_npc,r_npc[31:2]};
                r_carry <= w_carry;
                r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                r_shiftregb <= {w_in_shiftregb,r_shiftregb[31:2]};
                r_cnt <= r_cnt + 1;
            end else begin
                r_pc <= {r_npc[1:0],r_pc[31:2]};
                r_npc <= {w_to_r_npc,r_npc[31:2]};
                r_carry <= w_carry;
                r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                r_shiftregb <= {w_in_shiftregb,r_shiftregb[31:2]};
                r_cnt <= 1;
                r_npcadd_en <= 0;
                r_state <= `START_3;
            end
        end
    end



    /*********************************** START_3 ********************************************/
    always@(posedge w_clk) begin
        if(r_state==`START_3) begin
            if(w_alu_ctrl!=0) begin 
                r_state <= (w_op_imm) ? `ALUI_1 : `ALU_1;
            end 
            else if(w_shift_ctrl==1) r_state <= `SHIFTL_1;
            else if(w_shift_ctrl==2) r_state <= `SHIFTR_1;
            else if(w_bru_ctrl!=0) r_state <= `BRANCH_1;
            r_carry <= 0;
        end
    end

    /*********************************** EXECUTION ******************************************/
 
    /*********************************** ALU_1 **********************************************/
    wire [1:0] w_alu_result;
    wire [1:0] w_in_shiftrega,w_in_shiftregb;
    wire w_npcadd_en;

    m_ALU ALU0(w_alu_in1,w_alu_in2,r_carry,w_alu_ctrl,w_npcadd_en,w_alu_result,w_carry);

    always @(posedge w_clk) begin
        if(r_state==`ALU_1) begin
            if(r_cnt[4]==0) begin 
                case(w_alu_ctrl)
                    1 : begin //add
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                            r_carry <= w_carry;
                        end
                    2 : begin //sub
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                            r_carry <= w_carry;
                        end
                    3 : begin //xor
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                        end
                    4 : begin //or
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                        end
                    5 : begin //and
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                        end
                    default : begin 
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};
                        end
                endcase
                r_shiftregb <= {w_in_shiftregb,r_shiftregb[31:2]};
                r_cnt <= r_cnt + 1;
            end else begin
                case(w_alu_ctrl)
                    1 : begin //add
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                            r_carry <= w_carry;
                        end
                    2 : begin //sub
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                            r_carry <= w_carry;
                        end
                    3 : begin //xor
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                        end
                    4 : begin //or
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                        end
                    5 : begin //and
                            r_shiftrega <= {w_in_shiftrega,r_shiftrega[31:2]};
                        end
                    default : begin 
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};
                        end
                endcase
                r_shiftregb <= {w_in_shiftregb,r_shiftregb[31:2]};
                r_state <= `WRITEBACK_1;
                r_cnt <= 1;
            end
        end
    end    

    /*********************************** ALUI_1 *********************************************/
    always @(posedge w_clk) begin
        if(r_state==`ALUI_1) begin
            if(r_cnt[4]==0) begin 
                case(w_alu_ctrl)
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
                case(w_alu_ctrl)
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

    /*********************************** SHIFTR_1 *******************************************/
    always @(posedge w_clk) begin
        if(r_state==`SHIFTR_1) begin
            case(r_shiftregb)
                1 : begin 
                        r_cnt <= 0;
                        r_state <= `SHIFTR_2;
                    end
                2 : begin              
                        r_tmp <= r_shiftrega[1];
                        r_shiftrega <= {2'b00,r_shiftrega[31:2]};                
                        r_state <= `WRITEBACK_1;                   
                    end
                3 : begin
                        r_tmp <= r_shiftrega[1];
                        r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                        r_cnt <= 0;
                        r_state <= `SHIFTR_2;
                    end
                4 : begin
                        if(r_cnt!=2) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                5 : begin
                        if(r_cnt!=2) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                6 : begin
                        if(r_cnt!=3) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                7 : begin
                        if(r_cnt!=3) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                8 : begin
                        if(r_cnt!=4) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                9 : begin
                        if(r_cnt!=4) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                10 : begin
                        if(r_cnt!=5) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                11 : begin
                        if(r_cnt!=5) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                12 : begin
                        if(r_cnt!=6) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                13 : begin
                        if(r_cnt!=6) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                14 : begin
                        if(r_cnt!=7) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                15 : begin
                        if(r_cnt!=7) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                16 : begin
                        if(r_cnt!=8) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                17 : begin
                        if(r_cnt!=8) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                18 : begin
                        if(r_cnt!=9) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                19 : begin
                        if(r_cnt!=9) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                20 : begin
                        if(r_cnt!=10) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                21 : begin
                        if(r_cnt!=10) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                22 : begin
                        if(r_cnt!=11) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                23 : begin
                        if(r_cnt!=11) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                24 : begin
                        if(r_cnt!=12) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                25 : begin
                        if(r_cnt!=12) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                26 : begin
                        if(r_cnt!=13) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                27 : begin
                        if(r_cnt!=13) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                28 : begin
                        if(r_cnt!=14) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                29 : begin
                        if(r_cnt!=14) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                30 : begin
                        if(r_cnt!=15) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                31 : begin
                        if(r_cnt!=15) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 0;
                            r_state <= `SHIFTR_2;
                        end
                    end
                32 : begin
                        if(r_cnt==15) begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[1];
                            r_shiftrega <= {2'b00,r_shiftrega[31:2]};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                default : r_state <= `WRITEBACK_1;
            endcase
        end
    end

    /*********************************** SHIFTR_2 *******************************************/
    always @(posedge w_clk) begin
        if(r_state==`SHIFTR_2) begin
            if(r_cnt[4]==0) begin
                r_tmp <= r_shiftrega[1];
                r_shiftrega <= {r_shiftrega[0],r_tmp,r_shiftrega[31:2]};  
                r_cnt <= r_cnt + 1;
            end else begin
                r_tmp <= r_shiftrega[1];
                r_shiftrega <= {1'b0,r_tmp,r_shiftrega[31:2]};  
                r_cnt <= 1;
                r_state <= `WRITEBACK_1;
            end
        end
    end

    /*********************************** SHIFTL_1 *******************************************/

    always @(posedge w_clk) begin
        if(r_state==`SHIFTL_1) begin
            case(r_shiftregb)
                1 : begin
                        r_cnt <= 0;
                        r_state <= `SHIFTL_2;
                    end
                2 : begin
                        r_tmp <= r_shiftrega[30];
                        r_shiftrega <= {r_shiftrega[29:0],2'b00};
                        r_state <= `WRITEBACK_1;
                    end
                3 : begin
                        r_tmp <= r_shiftrega[30];
                        r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                        r_cnt <= 0;
                        r_state <= `SHIFTL_2;
                    end
                4 : begin
                        if(r_cnt!=2) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                5 : begin
                        if(r_cnt!=2) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                6 : begin
                        if(r_cnt!=3) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                7 : begin
                        if(r_cnt!=3) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                8 : begin
                        if(r_cnt!=4) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                9 : begin
                        if(r_cnt!=4) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                10 : begin
                        if(r_cnt!=5) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                11 : begin
                        if(r_cnt!=5) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                12 : begin
                        if(r_cnt!=6) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                13 : begin
                        if(r_cnt!=6) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                14 : begin
                        if(r_cnt!=7) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                15 : begin
                        if(r_cnt!=7) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                16 : begin
                        if(r_cnt!=8) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                17 : begin
                        if(r_cnt!=8) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                18 : begin
                        if(r_cnt!=9) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                19 : begin
                        if(r_cnt!=9) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                20 : begin
                        if(r_cnt!=10) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                21 : begin
                        if(r_cnt!=10) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                22 : begin
                        if(r_cnt!=11) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                23 : begin
                        if(r_cnt!=11) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                24 : begin
                        if(r_cnt!=12) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                25 : begin
                        if(r_cnt!=12) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                26 : begin
                        if(r_cnt!=13) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                27 : begin
                        if(r_cnt!=13) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                28 : begin
                        if(r_cnt!=14) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                29 : begin
                        if(r_cnt!=14) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                30 : begin
                        if(r_cnt!=15) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                31 : begin
                        if(r_cnt!=15) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 0;
                            r_state <= `SHIFTL_2;
                        end
                    end
                32 : begin
                        if(r_cnt==15) begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= r_cnt + 1;
                        end else begin
                            r_tmp <= r_shiftrega[30];
                            r_shiftrega <= {r_shiftrega[29:0],2'b00};  
                            r_cnt <= 1;
                            r_state <= `WRITEBACK_1;
                        end
                    end
                default : r_state <= `WRITEBACK_1;
            endcase
        end
    end

    /*********************************** SHIFTL_2 *******************************************/
    always @(posedge w_clk) begin
        if(r_state==`SHIFTL_2) begin
            if(r_cnt[4]==0) begin
                r_tmp <= r_shiftrega[30];
                r_shiftrega <= {r_shiftrega[29:0],r_tmp,r_shiftrega[31]};  
                r_cnt <= r_cnt + 1;
            end else begin
                r_tmp <= r_shiftrega[30];
                r_shiftrega <= {r_shiftrega[29:0],r_tmp,1'b0};  
                r_cnt <= 1;
                r_state <= `WRITEBACK_1;
            end
        end
    end

    /*********************************** BRANCH_1 *******************************************/

    /*********************************** WRITEBACK_1 ****************************************/
    always @(posedge w_clk) begin
        if(r_state==`WRITEBACK_1) begin
            r_shiftrega <= 0;
            r_shiftregb <= 32'd4;
            r_i_en <= 1;
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
        else if(r_state==`WRITEBACK_1) r_rout <= w_result;
    end

    /*********************************** WIREASSIGN *****************************************/
    assign w_i_addr = r_pc;
    wire [1:0] w_npc_2 = r_npc[1:0];
    wire [1:0] w_to_r_npc = (r_stall) ? w_npc_2 : w_alu_result;
    assign w_i_en = r_i_en;
    assign w_ir = w_i_in;

    assign w_npcadd_en = r_npcadd_en;

    wire [1:0] w_rrs1toshiftreg = f_mux_to_w_rrstoshiftreg(w_rrs1,(r_state==`START_2),r_cnt);
    wire [1:0] w_rrs2toshiftreg = f_mux_to_w_rrstoshiftreg(w_rrs2,(r_state==`START_2),r_cnt);
    wire [1:0] w_imm_2 = f_mux_to_w_imm_2(w_imm,r_cnt);

    wire [1:0] w_shiftrega_2 = r_shiftrega[1:0];
    wire [1:0] w_shiftregb_2 = r_shiftregb[1:0];

    wire [1:0] w_tmp_result = f_mux_to_w_tmp_result(r_shiftrega[0],r_shiftrega[31],r_tmp,r_state,r_cnt[4]);

    assign w_in_shiftrega = f_mux_to_w_in_shiftrega(w_rrs1toshiftreg,w_alu_result,w_tmp_result,r_state);
    assign w_in_shiftregb = f_mux_to_w_in_shiftregb(w_rrs2toshiftreg,w_imm_2,r_state,w_op_imm);

    wire [1:0] w_alu_in1 = f_mux_to_w_alu_in1(w_shiftrega_2,w_npc_2,r_state);
    wire [1:0] w_alu_in2 = f_mux_to_w_alu_in2(w_shiftregb_2,w_imm_2,r_state);

    wire [31:0] w_result = r_shiftrega;



    /*********************************** FUNCTIONS ******************************************/
    function [1:0] f_mux_to_w_imm_2;
        input [31:0] w_imm;
        input [5:0] w_sel;

        begin
            case(w_sel)
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

    function [1:0] f_mux_to_w_rrstoshiftreg;
        input [31:0] w_in;
        input w_en;
        input [5:0] w_sel;
        
        begin
            if(w_en) begin
                case(w_sel)
                    1 : f_mux_to_w_rrstoshiftreg = w_in[1:0];
                    2 : f_mux_to_w_rrstoshiftreg = w_in[3:2];
                    3 : f_mux_to_w_rrstoshiftreg = w_in[5:4];
                    4 : f_mux_to_w_rrstoshiftreg = w_in[7:6];
                    5 : f_mux_to_w_rrstoshiftreg = w_in[9:8];
                    6 : f_mux_to_w_rrstoshiftreg = w_in[11:10];
                    7 : f_mux_to_w_rrstoshiftreg = w_in[13:12];
                    8 : f_mux_to_w_rrstoshiftreg = w_in[15:14];
                    9 : f_mux_to_w_rrstoshiftreg = w_in[17:16];
                    10 : f_mux_to_w_rrstoshiftreg = w_in[19:18];
                    11 : f_mux_to_w_rrstoshiftreg = w_in[21:20];
                    12 : f_mux_to_w_rrstoshiftreg = w_in[23:22];
                    13 : f_mux_to_w_rrstoshiftreg = w_in[25:24];
                    14 : f_mux_to_w_rrstoshiftreg = w_in[27:26];
                    15 : f_mux_to_w_rrstoshiftreg = w_in[29:28];
                    16 : f_mux_to_w_rrstoshiftreg = w_in[31:30];
                    default : f_mux_to_w_rrstoshiftreg = 2'bxx;
                endcase
            end else begin
                f_mux_to_w_rrstoshiftreg = 2'b00;
            end
        end
    endfunction

    function [1:0] f_mux_to_w_in_shiftrega;
        input [1:0] w_rrs1toshiftreg;
        //input [1:0] w_shiftrega_2;
        input [1:0] w_alu_result;
        input [1:0] w_tmp_result;
        input [4:0] w_state;

        begin
            case(w_state)
                `START_2 : f_mux_to_w_in_shiftrega = w_rrs1toshiftreg;
                `ALU_1 : f_mux_to_w_in_shiftrega = w_alu_result;
                `ALUI_1 : f_mux_to_w_in_shiftrega = w_alu_result;
                `SHIFTR_2 : f_mux_to_w_in_shiftrega = w_tmp_result;
                `SHIFTL_2 : f_mux_to_w_in_shiftrega = w_tmp_result;
                default : f_mux_to_w_in_shiftrega = 2'b00;
            endcase
        end
    endfunction

    function [1:0] f_mux_to_w_in_shiftregb;
        input [1:0] w_rrs2toshiftreg;
        input [1:0] w_imm_2;
        //input [1:0] w_shiftregb_2;
        input [4:0] w_state;
        input w_op_imm;

        begin
            case(w_state)
                `START_2 : f_mux_to_w_in_shiftregb = (w_op_imm) ? w_imm_2 : w_rrs2toshiftreg;
                default : f_mux_to_w_in_shiftregb = 2'b00;
            endcase
        end
    endfunction

    function [1:0] f_mux_to_w_tmp_result;
        input w_shiftreg_0;
        input w_shiftreg_31;
        input w_tmp;
        input [4:0] w_state;
        input w_cnt_4;
        
        begin  
            case(w_state)
                `SHIFTR_2 : f_mux_to_w_tmp_result = (w_cnt_4==1) ? {1'b0,w_tmp} : {w_shiftreg_0,w_tmp};
                `SHIFTL_2 : f_mux_to_w_tmp_result = (w_cnt_4==1) ? {w_tmp,1'b0} : {w_tmp,w_shiftreg_31};
                default : f_mux_to_w_tmp_result = 2'b00;
            endcase
        end
    endfunction

    function [1:0] f_mux_to_w_alu_in1;
        input [1:0] w_shiftrega_2;
        input [1:0] w_npc_2;
        input [4:0] w_state;

        begin
            case(w_state)
                `START_2 : f_mux_to_w_alu_in1 = w_npc_2;
                default : f_mux_to_w_alu_in1 = w_shiftrega_2;
            endcase
        end

    endfunction

    function [1:0] f_mux_to_w_alu_in2;
        input [1:0] w_shiftregb_2;
        input [1:0] w_imm_2;
        input [4:0] w_state;
        
        begin
            case(w_state)
                `START_2 : f_mux_to_w_alu_in2 = w_shiftregb_2;
                `ALU_1 : f_mux_to_w_alu_in2 = w_shiftregb_2;
                `ALUI_1 : f_mux_to_w_alu_in2 = w_imm_2;
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

    reg [3:0] r_alu_ctrl0 = 0;
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
            `SHIFT_CTRL_SLL___ : begin
                r_shift_ctrl = 1;
                r_alu_ctrl = 0;
                end
            `SHIFT_CTRL_SRL___ : begin
                r_shift_ctrl = 2;
                r_alu_ctrl = 0;
                end
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

module m_regfile(w_clk, w_rs1, w_rs2, w_rdata1, w_rdata2, w_we, w_rd, w_wdata);
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

/********************************************************************************************/  

module m_ALU(w_alu_in1,w_alu_in2,w_carry_in,w_alu_ctrl,w_npcadd_en,w_alu_result,w_carry_out);
    input wire [1:0] w_alu_in1,w_alu_in2;
    input wire w_carry_in;
    input wire [9:0] w_alu_ctrl;
    input wire w_npcadd_en;
    output wire [1:0] w_alu_result;
    output wire w_carry_out;

    reg [1:0] r_alu_result;
    reg r_carry_out;

    always @(*) begin
        if(w_npcadd_en) begin
            {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} + {1'b0,w_alu_in2} + {2'b0,w_carry_in};
        end else begin
            case(w_alu_ctrl)
                1 : {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} + {1'b0,w_alu_in2} + {2'b0,w_carry_in};
                2 : {r_carry_out,r_alu_result} = {1'b0,w_alu_in1} - {1'b0,w_alu_in2} - {2'b0,w_carry_in};
                3 : r_alu_result = w_alu_in1 ^ w_alu_in2;
                4 : r_alu_result = w_alu_in1 | w_alu_in2;
                5 : r_alu_result = w_alu_in1 & w_alu_in2;
                default : r_alu_result = 0;
            endcase
        end
    end

    assign w_alu_result = r_alu_result;
    assign w_carry_out = r_carry_out;
endmodule