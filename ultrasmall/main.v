/********************************************************************************************/
`include "config.vh"
/********************************************************************************************/
`default_nettype none
/********************************************************************************************/

module main(w_clk,w_rst,w_rxd,r_txd,w_led);
    input wire w_clk,w_rst;
    input wire w_rxd;
    output reg r_txd;
    output wire [3:0] w_led;

    reg r_rst;
    always @(posedge w_clk) r_rst <= w_rst;

    reg r_core_rst;
    always @(posedge w_clk) r_core_rst <= (r_rst | !initdone);

    reg [31:0] r_scnt;
    always @(posedge w_clk) begin
        if(r_rst) r_scnt = 0;
        else r_scnt <= r_scnt + 1;
    end

    assign w_led = {r_scnt[24:22],r_rst};

    /*****************************************************************************************/
    reg r_rxd_t1,r_rxd_t2;
    always @(posedge w_clk) r_rxd_t1 <= w_rxd;
    always @(posedge w_clk) r_rxd_t2 <= r_rxd_t1;

    /*****************************************************************************************/
    wire [31:0] w_initdata;
    wire [31:0] w_initaddr;
    wire        w_initwe;
    wire        w_initdone;
    PLOADER ploader(w_clk, !r_rst, r_rxd_t2, w_initaddr, w_initdata, w_initwe, w_initdone);

    reg  [31:0] initdata;
    reg  [31:0] initaddr;
    reg  [3:0]  initwe;
    reg         initdone;
    always@(posedge w_clk) begin
        initdata <= (r_rst) ? 0 : (w_initwe) ? w_initdata   : 0;
        initaddr <= (r_rst) ? 0 : (initwe)   ? initaddr + 4 : initaddr;
        initwe   <= (r_rst) ? 0 : {4{w_initwe}};
        initdone <= (r_rst) ? 0 : w_initdone;
    end

    /****************************************************************************************/
    wire        w_halt;
    wire [31:0] w_rout, I_DATA, I_ADDR, D_DATA, WD_DATA, D_ADDR;
    wire [3:0]  D_WE;

    UltraSmall p(w_clk, !r_core_rst, w_rout, w_halt, I_ADDR, D_ADDR, I_DATA, D_DATA, WD_DATA, D_WE);

    wire [31:0] tmpdata;

    m_IMEM#(32,`MEM_SIZE/4) imem(w_clk, initwe[0], initaddr[$clog2(`MEM_SIZE)-1:2], 
                                 I_ADDR[$clog2(`MEM_SIZE)-1:2], initdata, I_DATA);

    m_DMEM#(32,`MEM_SIZE/4) dmem(w_clk, r_core_rst, initwe, initaddr[$clog2(`MEM_SIZE)-1:2], 
                                 initdata, tmpdata, w_clk, !r_core_rst, D_WE, 
                                 D_ADDR[$clog2(`MEM_SIZE)-1:2], WD_DATA, D_DATA);
                                 
    /****************************************************************************************/
    reg        r_D_ADDR;
    reg        r_D_WE;
    reg [31:0] r_WD_DATA;
    always@(posedge w_clk) begin
        if(r_rst) begin
            r_D_ADDR  <= 0;
            r_D_WE    <= 0;
            r_WD_DATA <= 0;
        end
        else begin
            r_D_ADDR  <= D_ADDR[15] & D_ADDR[30];
            r_D_WE    <= D_WE[0];
            r_WD_DATA <= WD_DATA;
        end
    end

    reg        tohost_we;
    reg [31:0] tohost_data;
    reg [7:0]  tohost_char;
    reg [1:0]  tohost_cmd;
    always@(posedge w_clk) begin
        if(r_rst) begin
            tohost_we   <= 0;
            tohost_data <= 0;
            tohost_char <= 0;
            tohost_cmd  <= 0;           
        end
        else begin
            tohost_we   <= (r_D_ADDR && (r_D_WE));
            tohost_data <= r_WD_DATA;
            tohost_char <= (tohost_we) ? tohost_data[7:0] : 0;
            tohost_cmd  <= (tohost_we) ? tohost_data[17:16] : 0;
        end
    end

    /****************************************************************************************/
    reg [7:0] squeue[0:`QUEUE_SIZE-1];
    reg  [$clog2(`QUEUE_SIZE)-1:0] queue_head = 0;
    reg  [$clog2(`QUEUE_SIZE)-1:0] queue_num  = 0;
    always@(posedge w_clk) begin
        if(r_rst) begin
            queue_head <= 0;
            queue_num <= 0;
        end
        else begin
            if(printchar) squeue[queue_addr] <= tohost_char;
            queue_head <= (!printchar & tx_ready & (queue_num > 0) & !uartwe) ? 
                           queue_head + 1 : queue_head;
            queue_num <= (printchar) ? queue_num + 1 : (tx_ready & (queue_num > 0) & !uartwe)
                          ? queue_num - 1 : queue_num;
        end
    end
    wire [$clog2(`QUEUE_SIZE)-1:0] queue_addr = queue_head + queue_num;
    wire printchar = (tohost_cmd==1);
    
    reg [7:0] uartdata;
    reg       uartwe;
    always@(posedge w_clk) begin
        if(r_rst) begin
            uartdata <= 0;
            uartwe <= 0;
        end
        else begin
            uartdata <= (!printchar & tx_ready & (queue_num > 0) & !uartwe) ? squeue[queue_head] : 0;
            uartwe   <= (!printchar & tx_ready & (queue_num > 0) & !uartwe) ? 1                  : 0;
        end
    end
    
    always@(posedge w_clk) r_txd <= w_txd;
    wire w_txd;
    wire tx_ready;
    UartTx UartTx0(w_clk, !r_core_rst, uartdata, uartwe, w_txd, tx_ready);
endmodule

/********************************************************************************************/
/* Program Loader: Initialize the main memory, copy memory image to the main memory         */
/********************************************************************************************/
module PLOADER (CLK, RST_X, RXD, ADDR, DATA, WE, DONE);
    input  wire        CLK, RST_X, RXD;
    output reg [31:0]  ADDR;
    output reg [31:0]  DATA;
    output reg         WE;
    output reg         DONE; // program load is done

    reg [31:0] waddr; // memory write address

    wire SER_EN;
    wire [7:0] SER_DATA;
    serialc serc (CLK, RST_X, RXD, SER_DATA, SER_EN);

    always @(posedge CLK) begin
        if(!RST_X) begin
            {ADDR, DATA, WE, waddr, DONE} <= 0;
        end else begin
            if(DONE==0 && SER_EN) begin
                ADDR  <= waddr;
                //ADDR  <= (waddr<32'h40000) ? waddr : {8'h04, 6'd0, waddr[17:0]};
                DATA  <= {SER_DATA, DATA[31:8]};
                WE    <= (waddr[1:0]==3);
                waddr <= waddr + 1;
            end else begin
                WE <= 0;
                if(waddr>=`MEM_SIZE) DONE <= 1;
            end
        end
    end
endmodule

/********************************************************************************************/
module UartTx(CLK, RST_X, DATA, WE, TXD, READY);
    input wire       CLK, RST_X, WE;
    input wire [7:0] DATA;
    output reg       TXD, READY;

    reg [8:0]   cmd;
    reg [11:0]  waitnum;
    reg [3:0]   cnt;

    always @(posedge CLK) begin
        if(!RST_X) begin
            TXD       <= 1'b1;
            READY     <= 1'b1;
            cmd       <= 9'h1ff;
            waitnum   <= 0;
            cnt       <= 0;
        end else if( READY ) begin
            TXD       <= 1'b1;
            waitnum   <= 0;
            if( WE )begin
                READY <= 1'b0;
                cmd   <= {DATA, 1'b0};
                cnt   <= 10;
            end
        end else if( waitnum >= `SERIAL_WCNT ) begin
            TXD       <= cmd[0];
            READY     <= (cnt == 1);
            cmd       <= {1'b1, cmd[8:1]};
            waitnum   <= 1;
            cnt       <= cnt - 1;
        end else begin
            waitnum   <= waitnum + 1;
        end
    end
endmodule

/********************************************************************************************/
/* RS232C serial controller (deserializer):                                                 */
/********************************************************************************************/
`define SS_SER_WAIT  'd0         // RS232C deserializer, State WAIT
`define SS_SER_RCV0  'd1         // RS232C deserializer, State Receive 0th bit
                                 // States Receive 1st bit to 7th bit are not used
`define SS_SER_DONE  'd9         // RS232C deserializer, State DONE
/********************************************************************************************/
module serialc(CLK, RST_X, RXD, DATA, EN);
    input  wire    CLK, RST_X, RXD; // clock, reset, RS232C input
    output [7:0]   DATA;            // 8bit output data
    output reg     EN;              // 8bit output data enable

    reg    [7:0]   DATA;
    reg    [3:0]   stage;
    reg    [12:0]  cnt;             // counter to latch D0, D1, ..., D7
    reg    [11:0]  cnt_start;       // counter to detect the Start Bit
    
    wire   [12:0]  waitcnt;
    assign waitcnt = `SERIAL_WCNT;

    always @(posedge CLK) begin
      if (!RST_X) cnt_start <= 0;
      else        cnt_start <= (RXD) ? 0 : cnt_start + 1;
    end
    always @(posedge CLK) begin
      if(!RST_X) begin
          EN     <= 0;
          stage  <= `SS_SER_WAIT;
          cnt    <= 1;
          DATA   <= 0;
      end else if (stage == `SS_SER_WAIT) begin // detect the Start Bit
          EN <= 0;
          stage <= (cnt_start == (waitcnt >> 1)) ? `SS_SER_RCV0 : stage;
      end else begin
          if (cnt != waitcnt) begin
              cnt <= cnt + 1;
              EN <= 0;
          end else begin               // receive 1bit data
              stage  <= (stage == `SS_SER_DONE) ? `SS_SER_WAIT : stage + 1;
              EN     <= (stage == 8)  ? 1 : 0;
              DATA   <= {RXD, DATA[7:1]};
              cnt <= 1;
          end
      end
    end
endmodule

/********************************************************************************************/
module m_IMEM#(parameter WIDTH=32, ENTRY=256)(CLK, WE, WADDR, RADDR, IDATA, ODATA);
    input  wire                     CLK, WE;
    input  wire [$clog2(ENTRY)-1:0] WADDR;
    input  wire [$clog2(ENTRY)-1:0] RADDR;
    input  wire [WIDTH-1:0]         IDATA;
    output reg  [WIDTH-1:0]         ODATA;
    
    reg [WIDTH-1:0] mem[0:ENTRY-1];
    always @(posedge CLK) begin
        if (WE) mem[WADDR] <= IDATA;
        ODATA <= mem[RADDR];
    end
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