`define START_PC 32'h00000000
`ifndef MEM_SIZE
`define MEM_SIZE 512*4            // Memory size in Byte
`endif
`define TOHOST_ADDR 32'h40008000 // TOHOST_ADDR
`define CMD_PRINT_CHAR 1         //
`define CMD_POWER_OFF  2         //
`define QUEUE_SIZE 32            // queue size of UART fifo
`define SERIAL_WCNT  20          // 100MHz clock and 5Mbaud -> 20
//`define NO_SERIAL
//`define ASYNC_IMEM
//`define ASYNC_DMEM

//state names
`define START_1 5'd0
`define START_2 5'd1
`define START_3 5'd2
`define ALU_1 5'd3
`define ALUI_1 5'd4
`define BRANCH_1 5'd5
`define JUMP_1 5'd6
`define SHIFTL_1 5'd7
`define SHIFTL_2 5'd8
`define SHIFTR_1 5'd9
`define SHIFTR_2 5'd10
`define SHIFTR_3 5'd11
`define LOADSTORE_1 5'd12
`define LOADSTORE_2 5'd13
`define LOAD_1 5'd14
`define LOAD_2 5'd15
`define STORE_1 5'd16
`define STORE_2 5'd17
`define WRITEBACK_1 5'd18

//select instruction
`define ALU_CTRL_ADD___ 4'h0
`define ALU_CTRL_SLT___ 4'h2
`define ALU_CTRL_SLTU__ 4'h3
`define ALU_CTRL_XOR___ 4'h4
`define ALU_CTRL_OR____ 4'h6
`define ALU_CTRL_AND___ 4'h7
`define ALU_CTRL_SUB___ 4'h8
`define ALU_CTRL_LUI___ 4'h9
`define ALU_CTRL_AUIPC_ 4'hA
`define SHIFT_CTRL_SLL___ 4'h1
`define SHIFT_CTRL_SRL___ 4'h5
`define SHIFT_CTRL_SRA___ 4'hD

//iverilog debug
//`define IVERILOG

//trace
`define TRACE
